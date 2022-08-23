RTOS summary

for passing structs (sensor data, etc) back and forth interrupt<->task: use Queues
for using sensors with interrupts: use binary semaphore (task blocks on semaphore posession)
for shared resource (such as radio transmit tasks): use mutex (for priority inheritance on more important tasks)

sensor demo:
timer task is scheduled every x ms with "interrupt", which gives semaphore
this unblocks a task "sensor reader", which is blocked on semaphore posession
task pushes data to a queue "sensor data queue"
"sensor data queue" data is read into master struct for current state in another task
timer task is scheduled with mutex to guard a "radio" which just prints to the console
timer task gives back mutex after it's done
second task is scheduled with "heartbeat" packets (highest priority) which also takes the "radio"

classes:

"sensor type": enum

"orientable": enum

"sensor data":

- a double value data
- an "orientable" that says what the value is - axis, or NED, or LATLNG, or TEMP
- a timestamp
- a crc code? todo for later

"queueable data":

- sensor data object
- sensor type

"sensor":

- a type
- a list of "sensor data" objects (with associated axes)
- a semaphore to control access via interrupt or timer
- string pointer "name"
