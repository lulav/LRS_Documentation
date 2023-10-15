---
toc_max_heading_level: 4
hide_title: true
sidebar_label: 'Batch content'
sidebar_position: 5
---

## Batch content

To get the overview about the exact batch, say 'dynamics', the function [**citros.info()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.info) is used:

```python
>>> citros.batch('dynamics').info()
```
It returns dictionary, that contains:
* 'size': size of the selected data
* 'sid_count': number of sids
* 'sid_list': list of the sids
* 'topic_count': number of topics
* 'topic_list': list of topics
* 'message_count': number of messages

The result is a [**CitrosDict**](../documentation/data_access/citros_dict.md#citros_data_analysis.data_access.citros_dict.CitrosDict) object, that inherits behaviour of an ordinary python dictionary, but has some additional methods.

<details>
  <summary>more about CitrosDict:</summary>

[**CitrosDict**](../documentation/data_access/citros_dict.md#citros_data_analysis.data_access.citros_dict.CitrosDict) object can be converted to json string by the method [**to_json()**](../documentation/data_access/citros_dict.md#citros_data_analysis.data_access.citros_dict.CitrosDict.to_json):

```python
>>> citros.batch('dynamics').info().to_json()
```
```js
{
  "size": "534 kB",
  "sid_count": 3,
  "sid_list": [
    1,
    2,
    3
  ],
  "topic_count": 4,
  "topic_list": [
    "A",
    "B",
    "C",
    "D"
  ],
  "message_count": 2000
}
```
or printed by the method [**print()**](../documentation/data_access/citros_dict.md#citros_data_analysis.data_access.citros_dict.CitrosDict.print):

```python
>>> citros.batch('dynamics').info().print()
```
```js
{
 'size': '534 kB',
 'sid_count': 3,
 'sid_list': [1, 2, 3],
 'topic_count': 4,
 'topic_list': ['A', 'B', 'C', 'D'],
 'message_count': 2000
}
```
</details>

:::note
It is not nessesary to call method [**batch()**](batch_overview.md#setting-batch) every time to set batch id or name. By parameter `inplace` the batch may be set to current [**CitrosDB**](getting_started.md#connection-to-the-database) object:
```python
>>> citros.batch('dynamics', inplace = True)
>>> print(f"current batch name: {citros.get_batch()}")
```

```text
current batch name: dynamics
```
:::

If specific sid is set, [**citros.info()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.info) also appends dictionary 'sids', with the following structure:
* 'sids':
    * int:
        * 'topics': 
            * str:
               * 'message_count': number of messages
               * 'start_time': time when simulation started
               * 'end_time': time when simulation ended
               * 'duration': duration of the simalation process
               * 'frequency': frequency of the simulation process ('message_count'/ 'duration', in Hz)

sid may be passed during [**CitrosDB** initialization](getting_started.md#connection-to-the-database) or by [**citros.sid()**](query_data.md#sid-constraints) method.

Let's assume that the batch has been already successfully set:

```python
>>> citros.batch('dynamics', inplace = True)
```

To get information about data with sid = 1 or 2 and print it:

```python
>>> citros.sid([1,2]).info().print()
```
```js
{
 'size': '365 kB',
 'sid_count': 2,
 'sid_list': [1, 2],
 'topic_count': 4,
 'topic_list': ['A', 'B', 'C', 'D'],
 'message_count': 1369,
 'sids': {
   1: {
     'topics': {
       'A': {
         'message_count': 155,
         'start_time': 312751159,
         'end_time': 100484965889,
         'duration': 100172214730,
         'frequency': 1.547
       },
       'B': {
         'message_count': 178,
         'start_time': 344594155,
         'end_time': 100752013600,
         'duration': 100407419445,
         'frequency': 1.773
       },
       'C': {
...
       }
     }
   }
 }
}
```

To set the specific topic, method [**topic()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.topic) is used.
This way, dictionary 'topics' is appended:
    
* 'topics':
    * str:
        * 'type': type
        * 'data_structure': structure of the data
        * 'message_count': number of messages

```python
>>> citros.topic('A').info().print()
```
```js
{
 'size': '126 kB',
 'sid_count': 3,
 'sid_list': [1, 2, 3],
 'topic_count': 1,
 'topic_list': ['A'],
 'message_count': 474,
 'topics': {
   'A': {
     'type': 'a',
     'data_structure': {
       'data': {
         'x': {
           'x_1': 'float',
           'x_2': 'float',
           'x_3': 'float'
         },
         'note': 'list',
         'time': 'float',
         'height': 'float'
       }
     },
     'message_count': 474
   }
 }
}
```

The specific piece of information may be achieved by the keywords of the [**CitrosDict**](../documentation/data_access/citros_dict.md#citros_data_analysis.data_access.citros_dict.CitrosDict), obtained by [**info()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.info) method.

<details>
  <summary>Examples</summary>

- Get total number of messages for sid = 1 or 2:

```python
#get information about sid equals 1 or 2 and assign the result to the variable 'inf_12':
inf_12 = citros.sid([1,2]).info()

#access number of messages by key 'message_count' and assign it to 'num':
num = inf_12['message_count']

#print:
print(f'total number of messages where sid = 1 or 2: {num}')
```
The result is:
```python
total number of messages where sid = 1 or 2: 1369
```

- Get number of messages for topic 'A' for each of these sids:
```python
#for sid equals 1, for topic 'A' get number of messages by 'message_count':
num_1 = inf_12['sids'][1]['topics']['A']['message_count']

#get numbe rof messages for sid = 2, topic 'A':
num_2 = inf_12['sids'][2]['topics']['A']['message_count']

#print both numbers:
print(f'Number of messages in "A" for sid = 1: {num_1}, for sid = 2: {num_2}')
```
The output:
```text
Number of messages in "A" for sid = 1: 155, for sid = 2: 164
```
- Get structure of the topic 'A' and the total number of messages for this topic:
```python
#get information about topic 'A' and assign the resulting dictionary to the variable 'inf_A'
inf_A = citros.topic('A').info()

#get number of messages by 'message_count' keyword:
num = inf_A['message_count']

#print number of messages:
print(f'total number of messages in topic "A": {num}')

#print structure of the json-data column:
inf_A['topics']['A']['data_structure']['data'].print()
```
The result is:
```js
total number of messages in topic "A": 474
{
 'x': {
   'x_1': 'float',
   'x_2': 'float',
   'x_3': 'float'
 },
 'note': 'list',
 'time': 'float',
 'height': 'float'
}
```
</details>


### Data structure

[**get_data_structure(topic = None)**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.get_data_structure) method of the [**CitrosDB**](getting_started.md#connection-to-the-database) object may be used to display json-data structure of the exact batch for the specific topics, set by [**topic**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.topic) or listed in `topic`:

```python
>>> citros.batch('dynamics').topic(['A', 'C']).get_data_structure()
```
```text
+-------+------+-----------------+
| topic | type | data            |
+-------+------+-----------------+
|     A |    a | {               |
|       |      |   x: {          |
|       |      |     x_1: float, |
|       |      |     x_2: float, |
|       |      |     x_3: float  |
|       |      |   },            |
|       |      |   note: list,   |
|       |      |   time: float,  |
|       |      |   height: float |
|       |      | }               |
+-------+------+-----------------+
|     C |    c | {               |
|       |      |   x: {          |
|       |      |     x_1: float, |
|       |      |     x_2: float, |
|       |      |     x_3: float  |
|       |      |   },            |
|       |      |   note: list,   |
|       |      |   time: float,  |
|       |      |   height: float |
|       |      | }               |
+-------+------+-----------------+
```

To get the result for all the existing topics, leave `topic` = None. Also let's set batch to this [**CitrosDB**](getting_started.md#connection-to-the-database) object in advance:

```python
>>> citros.batch('dynamics', inplace = True)
>>> citros.get_data_structure()
```
<details>
  <summary>The result:</summary>

```text
+-------+------+-----------------+
| topic | type | data            |
+-------+------+-----------------+
|     A |    a | {               |
|       |      |   x: {          |
|       |      |     x_1: float, |
|       |      |     x_2: float, |
|       |      |     x_3: float  |
|       |      |   },            |
|       |      |   note: list,   |
|       |      |   time: float,  |
|       |      |   height: float |
|       |      | }               |
+-------+------+-----------------+
|     B |    b | {               |
|       |      |   x: {          |
|       |      |     x_1: float, |
|       |      |     x_2: float, |
|       |      |     x_3: float  |
|       |      |   },            |
|       |      |   note: list,   |
|       |      |   time: float,  |
|       |      |   height: float |
|       |      | }               |
+-------+------+-----------------+
|     C |    c | {               |
|       |      |   x: {          |
|       |      |     x_1: float, |
|       |      |     x_2: float, |
|       |      |     x_3: float  |
|       |      |   },            |
|       |      |   note: list,   |
|       |      |   time: float,  |
|       |      |   height: float |
|       |      | }               |
+-------+------+-----------------+
```
</details>
