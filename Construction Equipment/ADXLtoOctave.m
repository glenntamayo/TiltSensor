pkg load instrument-controlif (exist("serial") == 3)   disp("Serial: Supported")else   disp("Serial: Unsupported")endifs = serial_setup("/dev/cu.usbserial-A9KNB9LX");get(s)