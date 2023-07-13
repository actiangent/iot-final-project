## Steps

* ```docker-compose run -d --service-ports```, with ```--service-ports``` option for exposing port. Based on this issue: https://github.com/docker/compose/issues/4799
* ```mosquitto_sub -h <host ip> -p 1883 -t <topic>``` for subscribing
* ```docker stop``` for stopping container

https://arduino.stackexchange.com/questions/19787/esp8266-analog-read-interferes-with-wifi