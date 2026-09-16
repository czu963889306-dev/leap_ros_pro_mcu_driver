#include "microros_internal.h"

#include <string.h>
#include <unistd.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lwip/inet.h"
#include "rmw_microros/custom_transport.h"
#include "uxr/client/profile/transport/custom/custom_transport.h"

#include "system_globals.h"

namespace microros {
namespace {

bool transport_open_udp(uxrCustomTransport *transport) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    ctx->fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (ctx->fd < 0) {
        return false;
    }

    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(ctx->local_port);
    bind(ctx->fd, reinterpret_cast<sockaddr *>(&local), sizeof(local));
    return true;
}

bool transport_close_udp(uxrCustomTransport *transport) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    if (ctx->fd >= 0) {
        close(ctx->fd);
        ctx->fd = -1;
    }
    return true;
}

size_t transport_write_udp(
    uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    const int sent = sendto(
        ctx->fd,
        buf,
        len,
        0,
        reinterpret_cast<sockaddr *>(&ctx->remote),
        sizeof(ctx->remote));
    return sent > 0 ? static_cast<size_t>(sent) : 0;
}

size_t transport_read_udp(
    uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(ctx->fd, &readfds);

    timeval tv = {};
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;
    const int ret = select(ctx->fd + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0) {
        return 0;
    }

    const int received = recv(ctx->fd, buf, len, 0);
    return received > 0 ? static_cast<size_t>(received) : 0;
}

bool ensure_uart_driver_initialized(UartTransportContext *ctx) {
    if (ctx == nullptr) {
        return false;
    }

    if (s_uart_transport_initialized && uart_is_driver_installed(ctx->port)) {
        return true;
    }

    const uart_config_t uart_config = {
        .baud_rate = net_cfg::kMicroRosUartBaudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };

    const bool driver_already_installed = uart_is_driver_installed(ctx->port);
    esp_err_t err = ESP_OK;
    if (!driver_already_installed) {
        err = uart_driver_install(
            ctx->port,
            net_cfg::kMicroRosUartRxBufferSize,
            net_cfg::kMicroRosUartTxBufferSize,
            0,
            nullptr,
            0);
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "failed to install UART%d driver: %s",
                     static_cast<int>(ctx->port),
                     esp_err_to_name(err));
            return false;
        }
    }

    err = uart_param_config(ctx->port, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "failed to configure UART%d: %s",
                 static_cast<int>(ctx->port),
                 esp_err_to_name(err));
        if (!driver_already_installed) {
            uart_driver_delete(ctx->port);
        }
        return false;
    }

    err = uart_set_pin(
        ctx->port,
        net_cfg::kMicroRosUartTxPin,
        net_cfg::kMicroRosUartRxPin,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "failed to set UART%d pins: %s",
                 static_cast<int>(ctx->port),
                 esp_err_to_name(err));
        if (!driver_already_installed) {
            uart_driver_delete(ctx->port);
        }
        return false;
    }

    uart_flush_input(ctx->port);
    s_uart_transport_initialized = true;
    return true;
}

bool transport_open_uart(uxrCustomTransport *transport) {
    auto *ctx = static_cast<UartTransportContext *>(transport->args);
    return ensure_uart_driver_initialized(ctx);
}

bool transport_close_uart(uxrCustomTransport *transport) {
    auto *ctx = static_cast<UartTransportContext *>(transport->args);
    return ctx != nullptr;
}

size_t transport_write_uart(
    uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *) {
    auto *ctx = static_cast<UartTransportContext *>(transport->args);
    if (ctx == nullptr || buf == nullptr || len == 0) {
        return 0;
    }

    const int written = uart_write_bytes(
        ctx->port,
        reinterpret_cast<const char *>(buf),
        len);
    return written > 0 ? static_cast<size_t>(written) : 0;
}

bool uart_start_filter_is_prefix(const uint8_t *bytes, size_t len) {
    if (bytes == nullptr || len >= sizeof(net_cfg::kMicroRosUartStartCommand)) {
        return false;
    }
    return memcmp(bytes, net_cfg::kMicroRosUartStartCommand, len) == 0;
}

size_t filter_uart_start_command_bytes(uint8_t *buf, size_t len) {
    if (buf == nullptr || len == 0) {
        return 0;
    }

    size_t out_len = 0;
    const size_t command_len = sizeof(net_cfg::kMicroRosUartStartCommand) - 1;
    for (size_t i = 0; i < len; ++i) {
        const uint8_t byte = buf[i];
        if (s_uart_start_filter_drop_delimiter) {
            if (byte == '\r' || byte == '\n') {
                continue;
            }
            s_uart_start_filter_drop_delimiter = false;
        }

        s_uart_start_filter_pending[s_uart_start_filter_pending_len++] = byte;
        while (s_uart_start_filter_pending_len > 0 &&
               !uart_start_filter_is_prefix(
                   s_uart_start_filter_pending,
                   s_uart_start_filter_pending_len)) {
            buf[out_len++] = s_uart_start_filter_pending[0];
            --s_uart_start_filter_pending_len;
            if (s_uart_start_filter_pending_len > 0) {
                memmove(
                    s_uart_start_filter_pending,
                    s_uart_start_filter_pending + 1,
                    s_uart_start_filter_pending_len);
            }
        }

        if (s_uart_start_filter_pending_len == command_len) {
            s_uart_start_filter_pending_len = 0;
            s_uart_start_filter_drop_delimiter = true;
        }
    }

    return out_len;
}

size_t transport_read_uart(
    uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *) {
    auto *ctx = static_cast<UartTransportContext *>(transport->args);
    if (ctx == nullptr || buf == nullptr || len == 0) {
        return 0;
    }

    const int wait_ms = timeout <= 0 ? 0 :
        (timeout > net_cfg::kMicroRosUartReadMaxWaitMs ? net_cfg::kMicroRosUartReadMaxWaitMs : timeout);
    const TickType_t ticks = pdMS_TO_TICKS(wait_ms);
    const int received = uart_read_bytes(ctx->port, buf, len, ticks);
    taskYIELD();
    return received > 0
        ? filter_uart_start_command_bytes(buf, static_cast<size_t>(received))
        : 0;
}

}  // namespace

bool setup_udp_transport() {
    memset(&s_udp_ctx, 0, sizeof(s_udp_ctx));
    s_udp_ctx.fd = -1;
    s_udp_ctx.local_port = net_cfg::kMicroRosLocalPort;
    s_udp_ctx.remote.sin_family = AF_INET;
    s_udp_ctx.remote.sin_port = htons(system_globals().runtime.microros_agent_port);

    if (inet_pton(AF_INET, system_globals().runtime.microros_agent_ip, &s_udp_ctx.remote.sin_addr.s_addr) != 1) {
        ESP_LOGE(kTag, "invalid micro-ROS agent IP: %s", system_globals().runtime.microros_agent_ip);
        return false;
    }

    rmw_uros_set_custom_transport(
        false,
        &s_udp_ctx,
        transport_open_udp,
        transport_close_udp,
        transport_write_udp,
        transport_read_udp);
    return true;
}

bool setup_uart_transport() {
    memset(&s_uart_ctx, 0, sizeof(s_uart_ctx));
    s_uart_ctx.port = net_cfg::kMicroRosUartPort;

    rmw_uros_set_custom_transport(
        true,
        &s_uart_ctx,
        transport_open_uart,
        transport_close_uart,
        transport_write_uart,
        transport_read_uart);
    return true;
}

bool wait_for_uart_start_command() {
    memset(&s_uart_ctx, 0, sizeof(s_uart_ctx));
    s_uart_ctx.port = net_cfg::kMicroRosUartPort;

    if (!ensure_uart_driver_initialized(&s_uart_ctx)) {
        return false;
    }

    ESP_LOGI(kTag, "micro-ROS UART waiting for host start command: %s",
             net_cfg::kMicroRosUartStartCommand);

    size_t matched = 0;
    uint32_t polls = 0;
    uint8_t bytes[64] = {};
    const size_t command_len = strlen(net_cfg::kMicroRosUartStartCommand);

    while (system_globals().runtime.wifi_comm_mode == WifiCommMode::kMicroRosUart) {
        const int received = uart_read_bytes(
            net_cfg::kMicroRosUartPort,
            bytes,
            sizeof(bytes),
            pdMS_TO_TICKS(net_cfg::kMicroRosUartStartReadWaitMs));

        for (int i = 0; i < received; ++i) {
            const char ch = static_cast<char>(bytes[i]);
            if (ch == net_cfg::kMicroRosUartStartCommand[matched]) {
                ++matched;
                if (matched >= command_len) {
                    s_uart_start_received = true;
                    s_uart_start_filter_pending_len = 0;
                    s_uart_start_filter_drop_delimiter = false;
                    uart_flush_input(net_cfg::kMicroRosUartPort);
                    ESP_LOGI(kTag, "micro-ROS UART start command received");
                    return true;
                }
            } else {
                matched = (ch == net_cfg::kMicroRosUartStartCommand[0]) ? 1 : 0;
            }
        }

        if ((++polls % net_cfg::kMicroRosUartStartLogPolls) == 0) {
            ESP_LOGI(kTag, "still waiting for micro-ROS UART host start command");
        }
    }

    return false;
}

}  // namespace microros
