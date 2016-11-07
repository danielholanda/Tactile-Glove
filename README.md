# Tactile Glove
A simple way of experimenting with the tactile internet. Build a tactile glove that let's you touch a virtual object and interact with it. 

## Demo
![Demo Doccou alpha](https://github.com/danielholanda/Tactile-Glove/raw/master/Media/demo.gif?raw=true)
## Hardware Required
* 1 Intel® Galileo (Gen1 or Gen2)
* 5 Coin flat vibrating motors (available on [Ebay](http://www.ebay.com/itm/10PCS-Coin-Flat-Vibrating-Micro-Motor-DC-3V-8mm-For-Pager-Cell-Phone-Mobile-NEW/152170487187?_trksid=p2047675.c100005.m1851&_trkparms=aid%3D222007%26algo%3DSIC.MBE%26ao%3D2%26asc%3D39823%26meid%3Da8ea03c553434c26bc03c3cd038e64d5%26pid%3D100005%26rk%3D1%26rkt%3D6%26sd%3D111945158509))
* 2 MPU6050 breakout boards
* 1 Power module (schematics comming soon)

## Dependencies
This project depends on the MRAA library on the Intel® Galileo. This library can be foud [here](https://github.com/intel-iot-devkit/mraa). This project also depends on toxilibs library on Processing. Toxilibs can be found [here](http://toxiclibs.org/downloads/).

## How to Use
### Processing
* Simply copy the project into your processing folder and run it. The hand will not move until you start the program at your Galileo.

### Galileo
* Copy the whole Galileo folder into one folder of the board running Linux (tested using Yocto).
* Open "tactileGlove.cpp" and change the IP address to the IP of the computer running the virtual hand.
    
    ```c++
    serverAddr.sin_addr.s_addr = inet_addr("YOUR.IP.HERE");
     ```
* Compile the project
    
    ```bash
    make
    ```
* Run the program:
    
    ```bash
    ./tactileGlove
    ```

## Bug Reports & Feature Requests
You can help by reporting bugs, suggesting features, reviewing feature specifications or just by sharing your opinion.
Use [GitHub Issues](https://github.com/danielholanda/Tactile-Glove/issues) for all of that.

## Contributing
1. Fork the project.
2. Create a branch for your new feature.
3. Test your code.
5. Submit a pull request.

All pull requests are welcome !

## Authors
This project was develloped by [Daniel Holanda](https://github.com/danielholanda/), José Cláudio and Matheus Torquato as part of the 2016 Brazilian Intel® Embedded Systems Competition. During this event, this project was awarded with the 3rd place of over 200 projects.

## License
This project uses the MIT license. See [LICENSE](https://github.com/danielholanda/Tactile-Glove/blob/master/LICENSE) for more details.
