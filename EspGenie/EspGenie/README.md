
espGenie
===========================================
This is a work in progress to attempt to be able to have a locally controlled switch with no external dependency on HomeGenie, IE the switch will still allow local switching without HomeGenie being up.


### About

This software is for allowing an ESP8266-12 module to function as a light switch or remote controlled socket, allowing for the status to be set locally or remotely (via MQTT)

The switch can be controlled locally or remotely and will send the status of the switch via MQTT to HomeGenie. Provision is made for querying the switch and also telling a light to flash (Security systems?)

### Building

## Install pre-requisites

- Install Arduino 1.6.8 from the [Arduino website](http://www.arduino.cc/en/main/software).
- Start Arduino and open Preferences window.
- Enter ```http://arduino.esp8266.com/stable/package_esp8266com_index.json``` into *Additional Board Manager URLs* field. You can add multiple URLs, separating them with commas.
- Open Boards Manager from Tools > Board menu and install *esp8266* platform (and don't forget to select your ESP8266 board from Tools > Board menu after installation).

## Install wifi Manager

 I have customised the WiFiManager library and the custom files are located here, however I never quite got them working through the IDE, so I overwrote the downloaded ones with these :)

 __Currently version 0.8+ works with release 2.0.0 or newer of the [ESP8266 core for Arduino](https://github.com/esp8266/Arduino)__
  - in Arduino IDE got to Sketch/Include Library/Manage Libraries
   ![Manage Libraries](http://i.imgur.com/9BkEBkR.png)

  - search for WiFiManager
   ![WiFiManager package](http://i.imgur.com/18yIai8.png)

  - click Install

## Compiling

- Compile and upload to your esp8266 module via a serial connection  

### Issues and support ###

I will try and help out where possible but I developed this for myself so I dont expect it to suit everyone, nor am I a developer!

### Contributing

Feel free to submit a pull request.

## Related projects

- https://github.com/genielabs/HomeGenie
- https://github.com/genielabs/mig-service-dotnet

### License Information

[READ LICENSE FILE](LICENSE)

### Disclaimer

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
