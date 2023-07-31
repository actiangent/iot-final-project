## Running Container

* ```docker run -d --name eclipse-mosquitto-docker -p 1883:1883 -v ${PWD}/config/mosquitto.conf:/mosquitto/config/mosquitto.conf -v ${PWD}/data:/mosquitto/data -v ${PWD}/log:/mosquitto/log eclipse-mosquitto```  

    or

* ```docker-compose up -d --build```, by using docker compose.
    <details>
    <summary>If service is not discoverable</summary>

    Try ```docker-compose run -d --service-ports eclipse-mosquitto```

    </details>

Enter container terminal
* ```docker exec -it eclipse-mosquitto sh```

## Eclipse Mosquitto
* ```mosquitto_sub -h <host ip> -p 1883 -t <topic>``` for subscribing
* ```mosquitto_pub -h <host ip> -p 1883 -t <topic> -m "message"``` for publishing