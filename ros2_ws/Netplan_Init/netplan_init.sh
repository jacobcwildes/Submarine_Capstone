#!/bin/bash

#This is in place to ensure that even if timing is not perfect on boot, the ethernet may still work

#Define IP addresses of both Pis
ip_address_pi1="192.168.1.69"
ip_address_pi2="192.168.1.70"

#Loop until Pis can ping each other

while : 
do
    if ping -c 1 -W 1 "$ip_address_pi1" && ping -c 1 -W 1 "$ip_address_pi2"; then
        echo "Both Pis can ping!"
        break
    else
        echo "Pis unable to ping, retrying in 5s"
        sleep 5 #Wait 5 seconds
        #Generate netplan config
        sudo netplan generate

        #Apply netplan config
        sudo netplan apply    
    fi
done


