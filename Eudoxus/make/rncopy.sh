#!/bin/bash

set -x

rncopyright_update.py  $@ -e openni -e primesense -e external -e examples -e ni
