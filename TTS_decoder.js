function Decoder(bytes, port) 
{
  var longitude_int, latitude_int, bat_int;
  var decoded = {};
  
  if (port === 2)
  {
    if(bytes[0]==9) // check if the header byte is 9.
    {
      latitude_int = (bytes[1] << 24) | (bytes[2] << 16) | (bytes[3] << 8) | (bytes[4]);
      decoded.latitude = latitude_int / 100000;
      longitude_int = (bytes[5] << 24) | (bytes[6] << 16) | (bytes[7] << 8) | (bytes[8]);
      decoded.longitude = longitude_int / 100000;
      bat_int = bytes[9];
      decoded.bat = bat_int / 10;
      return decoded;
    }
  }
}
