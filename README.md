squirrel_driver

You need to install the Robotino API2 debian package. See
http://wiki.openrobotino.org/index.php?title=API2

Linux i386 users edit /etc/apt/sources.list and add
```
deb http://doc.openrobotino.org/download/packages/i386 ./
```

Linux amd64 users edit /etc/apt/sources.list and add
```
deb http://doc.openrobotino.org/download/packages/amd64 ./
```

Then
```
sudp apt-get update
sudo apt-get install robotino-examples
sudo apt-get install robotino-examples
```

===============
[![Build Status](https://magnum.travis-ci.com/squirrel-project/squirrel_driver.svg?token=3yXoCRsCegowgzzpPuqw&branch=hydro_dev)](https://magnum.travis-ci.com/squirrel-project/squirrel_driver)
