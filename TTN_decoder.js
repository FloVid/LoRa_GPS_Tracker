function Decoder(bytes, port) {
  var lon = (bytes[4] << 24) | (bytes[5] << 16) |  (bytes[6] << 8) | bytes[7];
  var lat = (bytes[1] << 16) |  (bytes[2] << 8) | bytes[3];
  var bat = bytes[0];
  return {
    latitude: lat / 10000,
    longitude: lon / 10000,
    battery: bat / 10
  }
}
