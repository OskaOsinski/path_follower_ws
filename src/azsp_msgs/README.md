# Message package for AZSP

Repo containing a messages files for AZSP project

<details>
<summary markdown="span">Table of contents</summary>

- [Message package for AZSP](#message-package-for-azsp)  
    - [Project structre](#project-structre)
    - [Installation](#installation)
    - [Add new message](#add-new-message)
    - [Usage](#usage)
        - [Pyhon](#python)
        - [C++](#c)
    - [Contributor(s)](#contributors)

</details>

## Project structre

```bash
.
├── msg
│   ├── BoundingBox.msg # example message file
│   ├── ...
│   └── Obstacles.msg # example message file        
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Installation

```
cd && cd catkin_ws/src/
git clone https://github.com/Lukasiewicz-PIMOT/azsp_msgs.git
cd ..
catkin_make
```

## Add new message

To create new message file and load this message in other package you must create new file in msg directory (for example "Message.msg")

Next you must add this name to CMakeLists.txt in group: 
```
add_message_files(
    ... 
    Message.msg
    )
```
Next you must define Message file (Message.msg), for example:
```
Header header
int64 value
```
and you can compile package and use message in other package:
```
cd && cd catkin_ws && catkin_make
```
## Usage
how inport new message in other package:
### python
 ```
from azsp_msgs.msg import Message
...
message = Message()
...
node_pub = rospy.Publisher("/message_topic", Message, queue_size=1)
...
message.value = 2
node_pub.publish(message)
 ```
 ### C++
 ```
#include <azsp_msgs/Message.h>
...
azsp_msgs::Message message;
...
node_pub = n->advertise<azsp_msgs::Message>("/message_topic", 1);
...
message.value = 2;
node_pub.publish(message);
 ```

## Contributor(s)

[Mateusz Durau](mailto:mateusz.durau@pimot.lukasiewicz.gov.pl)
