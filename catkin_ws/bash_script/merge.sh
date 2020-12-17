#!/bin/bash

cat $(ls -1 *.py | grep -v webots_launcher )  > change.py
