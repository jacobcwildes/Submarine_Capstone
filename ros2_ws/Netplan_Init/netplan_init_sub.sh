#!/bin/bash

#This is in place to ensure that even if timing is not perfect on boot, the ethernet may still work for the sub

#Define IP addresses of both Pis
ip_address_pi1="192.168.1.69"
ip_address_pi2="192.168.1.70"

#Function to apply and generate netplan
apply_netplan(){
    sudo /usr/sbin/netplan generate >> /tmp/netplan_generate.log 2>&1 
    sudo /usr/sbin/netplan apply >> /tmp/netplan_apply.log 2>&1 
}

#Loop until Pis can ping each other

while : 
do
    if ping -c 1 -W 1 "$ip_address_pi1" && ping -c 1 -W 1 "$ip_address_pi2"; then
        echo "Both Pis can ping!" 
        exit 0
    else
        echo "Pis unable to ping, retrying in 5s" 
        sleep 5 #Wait 5 seconds
        #Generate and apply netplan config
         apply_netplan
    fi
done


