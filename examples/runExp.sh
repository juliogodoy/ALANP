#!/bin/bash

clear

echo "Starting experiment Circle.."
sh circle2017.sh
echo
sh congested2017.sh
echo
sh deadlock2017.sh
echo
sh bidi2017.sh
echo
sh inco2017.sh
echo
sh simple2017.sh
echo

