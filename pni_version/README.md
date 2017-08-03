This directory contains files used for version tracking that can be compiled into the host firmware.
this script can be run from either emb2 or windows with following environment
https://sourceforge.net/projects/mingw-w64/

The important files are:

## project.txt
This file contains just a one word unique project name consisting of characters from the following set { 0..9, a..z, - }. 
This project name a key that is used to generate the incremental build number so it needs to be a valid project in that system.

## version.txt
This file contains the semantic version MAJOR.MINOR.PATCH. One of these numbers should be incremented each time new changes are started.

From [Semantic Versioning 2.0.0](http://semver.org):

Given a version number MAJOR.MINOR.PATCH, increment the:

* MAJOR version when you make incompatible API changes,
* MINOR version when you add functionality in a backwards-compatible manner, and
* PATCH version when you make backwards-compatible bug fixes.

## build_number.txt
This file contains current build number. The build number should be incremented each time the source is compiled and there are uncommitted changes. The build number should be automatically generated but also stored here for reference.

## crc.txt
This file contains the CRCs for the generated firmware files
