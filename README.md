# RaceDestroyable

This repository demonstrates a threading issue with `rclpy` when accessing node lists in a non-thread-safe manner. Follow the instructions below to reproduce the error where the `NodeClient` script may raise an exception.

## Steps to Reproduce

### 1. Run the Service Server
`NodeServer.py` creates a dummy service server that returns a `Trigger.Response` immediately.

Open a terminal and run:
```bash
cd /path/to/your/directory
python3 NodeServer.py
```

### 2. Run the Service Client
`NodeClient.py` creates a service client that spawns 10 threads. Each thread creates a service client, makes a call, and destroys the client.

Open another terminal and run:
```bash
cd /path/to/your/directory
python3 NodeClient.py
```

After some iterations, the exception or a deadlock will occur, blocking the client.

## Expected Error
At some point, `NodeClient.py` will raise an exception similar to the following:

```bash
Traceback (most recent call last):
  File "/home/myLocalSuperUser/ros/Issue/NodeClient.py", line 75, in <module>
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

## Root Cause
The issue arises because `rclpy`'s `Executor` accesses node lists in a non-thread-safe manner. The problem can be seen in the following code snippets from `rclpy/Executor.py`:

### Example 1:
```python
for node in nodes_to_use:
  subscriptions.extend(filter(self.can_execute, node.subscriptions))
  timers.extend(filter(self.can_execute, node.timers))
  clients.extend(filter(self.can_execute, node.clients))
  services.extend(filter(self.can_execute, node.services))
  node_guards = filter(self.can_execute, node.guards)
  waitables.extend(filter(self.can_execute, node.waitables))
```

### Example 2:
```python
for client in node.clients:
  if client.handle.pointer in clients_ready:
    if client.callback_group.can_execute(client):
      handler = self._make_handler(client, node, self._take_client, self._execute_client)
    yielded_work = True
    yield handler, client, node
```

### Solution
To prevent these errors, locks should be applied to the node lists to ensure they are not modified while being accessed by other threads.
