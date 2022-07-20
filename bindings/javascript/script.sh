# install dependencies for JS: this command will create the directory node_modules/ (should not be pushed into the repo)
npm install

sudo apt  install protobuf-compiler
# compile protobuf for JS: buffer_pb.js (should not be pushed into the repo)
protoc --js_out=import_style=commonjs,binary:. ./network/buffer.proto

npm install browserify -g
# compile the JS scripts into a single script: bundle.js (should not be pushed into the repo)
browserify network/network.js network/network_sensor_enumerator.js network/network_depth_sensor.js  network/network_storage.js  network/network_temperature_sensor.js sdk/system.js sdk/camera.js sdk/calibration.js sdk/frame.js -o bundle.js

# start a web server in localhost on port 9000
python3 -m http.server 9000 &

# open an incognito window (no cache) in google chrome browser showing an example web page
google-chrome --incognito http://localhost:9000/examples/example.html
