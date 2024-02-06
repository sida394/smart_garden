| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

# HTTP server 

## Example Output

```
I (8596) example: Starting server
I (8596) esp_https_server: Starting server
I (8596) esp_https_server: Server listening on port 443
I (8596) example: Registering URI handlers
I (8606) esp_netif_handlers: example_connect: sta ip: 192.168.2.3, mask: 255.255.255.0, gw: 192.168.2.1
I (8616) example_connect: Got IPv4 event: Interface "example_connect: sta" address: 192.168.194.219
I (9596) example_connect: Got IPv6 event: Interface "example_connect: sta" address: fe80:0000:0000:0000:266f:28ff:fe80:2c74, type: ESP_IP6_ADDR_IS_LINK_LOCAL
I (9596) example_connect: Connected to example_connect: sta
W (9606) wifi:<ba-add>idx:0 (ifx:0, ee:6d:19:60:f6:0e), tid:0, ssn:2, winSize:64
I (9616) example_connect: - IPv4 address: 192.168.194.219
I (9616) example_connect: - IPv6 address: fe80:0000:0000:0000:266f:28ff:fe80:2c74, type: ESP_IP6_ADDR_IS_LINK_LOCAL
W (14426) wifi:<ba-add>idx:1 (ifx:0, ee:6d:19:60:f6:0e), tid:4, ssn:0, winSize:64
I (84896) esp_https_server: performing session handshake
```
