#!/bin/sh

gnome-terminal --tab -e  "   ./Core 3 " 
sleep 5
gnome-terminal --tab -e "echo running   1  ./Core 2" 
sleep 5
gnome-terminal --tab -e  "   ./Core 1" 
sleep 5
gnome-terminal --tab -e  "   ./Core 0"
