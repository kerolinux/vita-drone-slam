the drone IP and video port (192.168.4.153:8080)
the start bytes 42 76 and stop bytes 42 77
the packet layout: 8-byte header, with frame ID, last flag, packet count, and the “TZH” marker at bytes 4-6
frames are JPEGs cut out between FF D8 and FF D9
the finding that the stream got stuck after a failed run and a power cycle fixed it
the packet loss numbers above
credit to the opendrone repo as the source of the handshake