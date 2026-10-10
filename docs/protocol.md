Video testing:
the drone IP and video port (192.168.4.153:8080)
the start bytes 42 76 and stop bytes 42 77
the packet layout: 8-byte header, with frame ID, last flag, packet count, and the “TZH” marker at bytes 4-6
frames are JPEGs cut out between FF D8 and FF D9
the finding that the stream got stuck after a failed run and a power cycle fixed it
the packet loss numbers above
credit to the opendrone repo as the source of the handshake

Stream measurements:
Measure	     Result
Resolution	640x480
Complete frame rate	about 8-9 fps (11 fps in the first window, then settled)
Complete frames	about 77-84%
Frame size	about 20 KB average (JPEG)
