# install dependencies for JS
npm install

# compile protobuf for JS: buffer_pb.js
protoc --js_out=import_style=commonjs,binary:. buffer.proto

# compile the JS scripts into a single script: bundle.js
browserify network/network.js network/network_sensor_enumerator.js network/network_depth_sensor.js  network/network_storage.js  network/network_temperature_sensor.js -o bundle.js

# start a web server in localhost on port 9000
python3 -m http.server 9000 &

# open an incognito window (no cache) in google chrome browser showing an example web page
google-chrome --incognito http://localhost:9000/example.html
