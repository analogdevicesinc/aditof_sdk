python3 -m http.server 9000 &
watchify network/network.js network/network_sensor_enumerator.js network/network_depth_sensor.js  network/network_storage.js  network/network_temperature_sensor.js -o bundle.js &
ps
google-chrome --incognito http://localhost:9000/example.html

