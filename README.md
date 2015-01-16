Summary
=======

Library Module for GT-511C Fingerprint Reader
=====================================
by Joe Kroesche (kroesche.org)

---

License
-------
Copyright &copy; 2015, Joseph Kroesche (kroesche.org).  All rights reserved.
This software is released under the FreeBSD license, found in the accompanying file
LICENSE.txt and at the following URL:
  http://www.freebsd.org/copyright/freebsd-license.html
This software is provided as-is and without warranty.

Summary
=======
This library module is meant to work with a GT-511C1R fingerprint reader.  This is an
inexpensive fingerprint reader module that is available at hobbyist vendors such as
Sparkfun.  For example, here is the link for the product on Sparkfun at the time of
this writing:

https://www.sparkfun.com/products/13007

The library provides functions to control the various features of the fingerprint
scanner.  Primarily it makes it easy to add, remove, and verify fingerprints.  It is
designed to be portable - there is no direct hardware access coded into the driver.
All hardware access is performed through callback functions that must be provided
by the application.

This code should be portable to many embedded systems, although I tested it on a
Silicon Labs EFM32 Wonder Gecko:

http://www.silabs.com/products/mcu/lowpower/Pages/efm32wg-wonder-gecko.aspx

I used the GT-511C1R fingerprint module mentioned above, but I think this library
might also work with other models of this fingerprint reader such as GT-511C3, but
I have not tested that.

Hopefully this is obvious, but this is not a complete application, it is only a
library module that is used within an application.

Documentation
=============
The Doxygen-generated API documentation can be found at http://kroesche.github.io/fingerprint_gt511/

