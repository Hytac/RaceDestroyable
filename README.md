# RaceDestroyable

Clone the repository and run both scripts, at some point the NodeClient will get an exception.

```bash
Traceback(most recent call last):
  File "/home/myLocalSuperUser/ros/Issue/NodeClient.py", line 75, in <module >
  loop.run_until_complete(main())
  File "/home/myLocalSuperUser/ros/Issue/NodeClient.py", line 67, in main
  executor.spin()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 294, in spin
  self.spin_once()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 795, in spin_once
  self._spin_once_impl(timeout_sec)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 792, in _spin_once_impl
  future.result()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 94, in result
  raise self.exception()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 239, in __call__
  self._handler.send(None)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 430, in handler
  arg = take_from_wait_list(entity)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 365, in _take_client
  with client.handle:
rclpy._rclpy_pybind11.InvalidHandle: cannot use Destroyable because destruction was requested
```

rclpy/Executor.py acces to the node liss in a non thread-safe way.

```python 
for node in nodes_to_use:
  subscriptions.extend(filter(self.can_execute, node.subscriptions))
  timers.extend(filter(self.can_execute, node.timers))
  clients.extend(filter(self.can_execute, node.clients))
  services.extend(filter(self.can_execute, node.services))
  node_guards = filter(self.can_execute, node.guards)
  waitables.extend(filter(self.can_execute, node.waitables))
```

or 

```python 
for client in node.clients:
  if client.handle.pointer in clients_ready:
    if client.callback_group.can_execute(client):
      handler = self._make_handler(client, node, self._take_client, self._execute_client)
    yielded_work = True
    yield handler, client, node
```

Lock to the list should be done to prevent the list to be changed while its being used by others.


# Reproduce

NodeServer.py will create a dummy service server that return a Trigger.Response immediately.

In one terminal
```bash
cd toTheDirectory
python3 NodeServer.py
```

NodeClient.py will create a service client that creates 10 threads, every thread creates a service client, make a call and destroy the client.

In other terminal
```bash
cd toTheDirectory
python3 NodeClient.py
```
In some iterations the exception will appear or a deadlock will block the client.
