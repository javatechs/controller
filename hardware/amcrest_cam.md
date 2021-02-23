# AmCrest Camera

## Installs
- [Install remo.tv controller](https://github.com/javatechs/controller) software.
- [Install Amcrest Python interface](https://github.com/tchellomello/python-amcrest/).

## Configure controller.conf
- **type** or 'hardware' type should be amcrest_cam.

```
       type=amcrest_cam
```

- **host** is the machine with the rosbridge node.
- **port** is the machine with the rosbridge node.
- **user_slice** is the machine with the rosbridge node.
- **uid** camera user id.
- **pwd** camera password.

```
	##
	# Amcrest settings
	##
	[amcrest]
	host = 10.0.0.1
	port = 80
	user_slice = 60
	uid=xx
	pwd=xx
```
## Try Amcrest camera & remo
- Start [remo.tv](https://github.com/javatechs/controller) controller

```
$ ~/remotv$ python controller.py
```
Be sure Amcrest camera is online.

## Reference Links
* [Amcrest Python interface repository](https://github.com/tchellomello/python-amcrest/)
* [Amcrest Python interface documentation](https://python-amcrest.readthedocs.io/)
* [Amcrest Python API documentation](https://roslibpy.readthedocs.io/en/latest/reference/index.html)
