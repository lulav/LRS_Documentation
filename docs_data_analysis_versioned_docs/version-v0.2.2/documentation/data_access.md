---
# Display h3 headings
toc_max_heading_level: 3
sidebar_label: 'Documentation'
description: 'Documentation'
hide_title: true
---



    
## Module `citros_data_analysis.data_access` {#citros_data_analysis.data_access}






    
## Function `get_version` {#citros_data_analysis.data_access.get_version}




```python
def get_version()
```


<details>
  <summary>Description</summary>

Return version of the citros_data_analysis package.

---
#### Examples

```python
>>> from citros_data_analysis import data_access as da
>>> print(da.get_version())
v0.1.1
```

</details>



    
## Class `CitrosDB` {#citros_data_analysis.data_access.CitrosDB}





```python
class CitrosDB(
    host=None,
    user=None,
    password=None,
    database=None,
    schema=None,
    batch=None,
    port=None,
    sid=None,
    debug_flag=False
)
```


<details>
  <summary>Description</summary>

CitrosDB object, that allows to get general information about the batch and make queries.

---
#### Parameters

**```host```** :&ensp;**str**
:   Database host adress.
    If None, uses predefined ENV parameter "PG_HOST".


**```user```** :&ensp;**str**
:   User name.
    If not specified, uses predefined ENV parameter "PG_USER".


**```password```** :&ensp;**str**
:   Password.
    If not specified, uses predefined ENV parameter "PG_PASSWORD".


**```database```** :&ensp;**str**
:   Database name.
    If not specified, uses predefined ENV parameter "PG_DATABASE".


**```schema```** :&ensp;**str**
:   If None, uses predefined ENV parameter "PG_SCHEMA".


**```batch```** :&ensp;**str**
:   Batch name.
    If not specified, uses predefined ENV parameter "bid".


**```port```** :&ensp;**str**
:   If None, uses predefined ENV parameter "PG_PORT".


**```sid```** :&ensp;**int**, optional
:   Default sid.
    If not specified, uses predefined ENV parameter "CITROS_SIMULATION_RUN_ID" if it exists or None.


**```debug_flag```** :&ensp;**bool**, default **False**
:   If False, program will try to handle errors and only print error messages without code breaking.
    If True, this will cause the code to abort if an error occurs.





</details>






    
### Method `avg` {#citros_data_analysis.data_access.CitrosDB.avg}




```python
def avg(
    self,
    n_avg=None
)
```


<details>
  <summary>Description</summary>

Average **n_avg** number of messages.

Messages with different sids are processed separetly. 
The value in 'rid' column is set as a minimum value among the 'rid' values of the averaged rows.

---
#### Parameters

**```n_avg```** :&ensp;**int**
:   Number of messages to average.

---
#### Returns

&ensp;**[CitrosDB](#citros_data_analysis.data_access.CitrosDB "citros_data_analysis.data_access.CitrosDB")**
:   CitrosDB with parameters set for sampling method 'avg'.

---
#### Examples

To average each 3 messages of the topic 'A' and get the result:

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').avg(3).data()
```

</details>


    
### Method `data` {#citros_data_analysis.data_access.CitrosDB.data}




```python
def data(
    self,
    data_names=None
)
```


<details>
  <summary>Description</summary>

Return table with data.
 
Query data according to the constraints set by topic(), rid(), sid() and time() methods
and one of the aggrative method skip(), avg() or move_avg().

---
#### Parameters

**```data_names```** :&ensp;**list**, optional
:   Labels of the columns from json data column.

---
#### Returns

&ensp;**pandas.DataFrame**
:   Table with selected data.

---
#### Examples

if the structure of the data column is the following:

```python
{x: {x_1: 1}, note: ['a', 'b']}
{x: {x_1: 2}, note: ['c', 'd']}
...
```
to get the column with the values of json-object 'x_1'
and the column with the values from the first position in the json-array 'note':

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').data(["data.x.x_1", "data.note[0]"])
```


To get all the whole 'data' column with json-objects divide into separate columns:

```python
>>> df = citros.topic('A').data()
```


To get the whole 'data' column as a json-object:

```python
>>> df = citros.topic('A').data(["data"])
```

</details>


    
### Method `get_batch_size` {#citros_data_analysis.data_access.CitrosDB.get_batch_size}




```python
def get_batch_size(
    self
)
```


<details>
  <summary>Description</summary>

Print sizes of the all tables in the current schema.

Print table with batch name, batch size and total batch size with indexes.

---
#### Examples

Print the table with information about batch sizes:

```python
>>> citros = da.CitrosDB()
>>> citros.get_batch_size()
+--------------------------+------------+------------+
| batch                    | size       | total size |
+--------------------------+------------+------------+
| stars                    | 32 kB      | 64 kB      |
| galaxies                 | 8192 bytes | 16 kB      |
+--------------------------+------------+------------+
```

</details>


    
### Method `get_counts` {#citros_data_analysis.data_access.CitrosDB.get_counts}




```python
def get_counts(
    self,
    column_name,
    group_by=None,
    filter_by=None
)
```


<details>
  <summary>Description</summary>

Return number of the rows in the column **column_name**.

---
#### Parameters

**```column_name```** :&ensp;**str**
:   Label of the column.


**```group_by```** :&ensp;**list**, optional
:   Labels of the columns to group by. If blank, do not group.


**```filter_by```** :&ensp;**dict**, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by **topic()**, **rid()**, **sid()**, **time()** and **set_filter()** and will override them.

---
#### Returns

&ensp;**list** of **tuples** or **None**
:   Number of rows in **column_name**.

---
#### Examples

If the structure of the data column is the following:

```python
{x: {x_1: 52}, note: ['b', 'e']}
{x: {x_1: 11}, note: ['a', 'c']}
{x: {x_1: 92}, note: ['b', 'd']}
...
```
to the number of values from the first position of the json-array 'note' for topics 'A' or 'B',
where 10 <= 'time' <= 5000 and data.x.x_1 > 10:

```python
>>> citros = da.CitrosDB()
>>> citros.set_filter({'topic': ['A', 'B'], 'data.x.x_1': {'gt': 10}})\
...       .time(start = 10, end = 5000)\
...       .get_counts('data.note[0]')
[(30,)]
```


To perform under the same conditions, but to get values grouped by topics:

```python
>>> citros.set_filter({'topic': ['A', 'B'], 'data.x.x_1': {'gt': 10}})\
...       .time(start = 10, end = 5000)\
...       .get_counts('data.note[0]', group_by = ['topic'])
[('A', 17), ('B', 13)]
```


The same, but passing all constraintes by **filter_by** parameter:

```python
>>> citros.get_counts('data.note[0]',
...                    group_by = ['topic'],
...                    filter_by = {'topic': ['A', 'B'], 
...                                 'time': {'gte': 10, 'lte': 5000}, 
...                                 'data.x.x_1' : {'gt':10}})
[('A', 17), ('B', 13)]
```

</details>


    
### Method `get_data_structure` {#citros_data_analysis.data_access.CitrosDB.get_data_structure}




```python
def get_data_structure(
    self,
    topic=None
)
```


<details>
  <summary>Description</summary>

Print structure of the json-data column for the specific topic(s).

Each tuple conatains topic and type names, structure of the corresponding data and number of sids.

---
#### Parameters

**```topic```** :&ensp;**list** or **list** of **str**, optional
:   list of the topics to show data structure for.
    Have higher priority, than those defined by **topic()** and **set_filter()** methods 
    and will override them.
    If not specified, shows data structure for all topics.

---
#### Examples

Print structure of the json-data column for topics 'A' and 'C':

```python
>>> citros = da.CitrosDB()
>>> citros.get_data_structure(['A', 'C'])
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

</details>


    
### Method `get_max_value` {#citros_data_analysis.data_access.CitrosDB.get_max_value}




```python
def get_max_value(
    self,
    column_name,
    filter_by=None
)
```


<details>
  <summary>Description</summary>

Return maximum value of the column **column_name**.

---
#### Parameters

**```column_name```** :&ensp;**str**
:   Label of the column.


**```filter_by```** :&ensp;**dict**, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by **topic()**, **rid()**, **sid()**, **time()** and **set_filter()** and will override them.

---
#### Returns

&ensp;**int, float, str** or **None**
:   Maximum value of the column **column_name**.

---
#### Examples

Get max value of the column 'rid' where topics are 'A' or 'B', 10 <= 'time' <= 5000 and data.x.x_1 > 10:

```python
>>> citros = da.CitrosDB()
>>> result = citros.set_filter({'topic': ['A', 'B'], 'data.x.x_1': {'gte': 10}})\
...                .time(start = 10, end = 5000)\
...                .get_max_value('rid')
>>> print(result)
163
```


The same, but passing all constraintes by **filter_by** parameter:

```python
>>> result = citros.get_max_value('rid',
...                               filter_by = {'topic': ['A', 'B'], 
...                                            'time': {'gte': 10, 'lte': 5000}, 
...                                            'data.x.x_1' : {'gt':10}})
>>> print(result)
163
```

</details>


    
### Method `get_min_value` {#citros_data_analysis.data_access.CitrosDB.get_min_value}




```python
def get_min_value(
    self,
    column_name,
    filter_by=None
)
```


<details>
  <summary>Description</summary>

Return minimum value of the column **column_name**.

---
#### Parameters

**```column_name```** :&ensp;**str**
:   Label of the column.


**```filter_by```** :&ensp;**dict**, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by **topic()**, **rid()**, **sid()**, **time()** and **set_filter()** and will override them.

---
#### Returns

&ensp;**int, float, str** or **None**
:   Minimum value of the column **column_name**.

---
#### Examples

Get min value of the column 'rid' where topics are 'A' or 'B', 10 <= 'time' <= 5000 and data.x.x_1 > 10:

```python
>>> citros = da.CitrosDB()
>>> result = citros.set_filter({'topic': ['A', 'B'], 'data.x.x_1': {'gte': 10}})\
...                .time(start = 10, end = 5000)\
...                .get_min_value('rid')
>>> print(result)
13
```


The same, but passing all constraintes by **filter_by** parameter:

```python
>>> result = citros.get_min_value('rid',
...                               filter_by = {'topic': ['A', 'B'], 
...                                            'time': {'gte': 10, 'lte': 5000}, 
...                                            'data.x.x_1' : {'gt':10}})
>>> print(result)
13
```

</details>


    
### Method `get_sid_tables` {#citros_data_analysis.data_access.CitrosDB.get_sid_tables}




```python
def get_sid_tables(
    self,
    data_query=None,
    topic=None,
    additional_columns=None,
    filter_by=None,
    order_by=None,
    method=None,
    n_avg=1,
    n_skip=1
)
```


<details>
  <summary>Description</summary>

Return dict of tables, each of the tables corresponds to exact value of sid.

---
#### Parameters

**```data_query```** :&ensp;**list**, optional
:   Labels of the data to download from the json-format column "data".
    If blank list, then all columns are are downloaded.


**```topic```** :&ensp;**str**
:   Name of the topic.
    Have higher priority than defined by **topic()**.
    May be overrided by **filter_by** argument.


**```additional_columns```** :&ensp;**list**, optional
:   Columns to download outside the json "data".
    If blank list, then all columns are are downloaded.


**```filter_by```** :&ensp;**dict**, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by **topic()**, **rid()**, **sid()**, **time()** and **set_filter()** and will override them.


**```order_by```** :&ensp;**dict**, optional
:   Keys must match labels of the columns, values specify ascending ('asc') or descending ('desc') order.
    Conditions, passed here, have higher priority over those defined by **set_order()** and will override them.


**```method```** :&ensp;`{'', 'avg', 'move_avg', 'skip'}`, optional
:   Method of sampling:
    'avg' - average - average **n_avg** rows;
    'move_avg' - moving average - average **n_avg** rows and return every **n_skip**-th row;
    'skip' - skiping **n_skip** rows;
    '' - no sampling.
    If not specified, no sampling is applied


**```n_avg```** :&ensp;**int**
:   Used only if **method** is 'move_avg' or 'avg'.
    Number of rows for averaging.


**```n_skip```** :&ensp;**int**
:   Used only if **method** is 'move_avg' or 'skip'.
    Number of rows to skip in a result output. 
    For example, if skip = 2, only every second row will be returned.

---
#### Returns

&ensp;**dict** of **pandas.DataFrames**
:   dict with tables, key is a value of sid.

---
#### Examples

Download averaged data for each sid separetly, setting ascending order by 'rid':

```python
>>> citros = da.CitrosDB()
>>> dfs = citros.topic('A').set_order({'rid': 'asc'}).avg(2)\
                .get_sid_tables(data_query=['data.x.x_1'])
```


Print sid value:

```python
>>> print(f'sid values are: {list(dfs.keys())}')
sid values are: [1, 2, 3, 4]
```


Get table corresponding to the sid = 2 and assign it to 'df':

```python
>>> df = dfs[2]
```


The same, but setting constraints by parameters: 

```python
>>> dfs = citros.get_sid_tables(data_query = ['data.x.x_1'],
...                             topic = 'A', 
...                             additional_columns = [], 
...                             filter_by = {}, 
...                             order_by = {'rid': 'asc'}, 
...                             method = 'avg', 
...                             n_avg = 2)
>>> print(f'sid values are: {list(dfs.keys())}')
sid values are: [1, 2, 3, 4]
```

</details>


    
### Method `get_unique_counts` {#citros_data_analysis.data_access.CitrosDB.get_unique_counts}




```python
def get_unique_counts(
    self,
    column_name,
    group_by=None,
    filter_by=None
)
```


<details>
  <summary>Description</summary>

Return number of the unique values in the column **column_name**.

---
#### Parameters

**```column_name```** :&ensp;**str**
:   Column to count its unique values.


**```group_by```** :&ensp;**list**, optional
:   Labels of the columns to group by. If blank, do not group.


**```filter_by```** :&ensp;**dict**, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by **topic()**, **rid()**, **sid()**, **time()** and **set_filter()** and will override them.

---
#### Returns

&ensp;**int** or **list** or **None**
:   Counts of the unique values in **column_name**.

---
#### Examples

If the structure of the data column is the following:

```python
{x: {x_1: 52}, note: ['b', 'e']}
{x: {x_1: 11}, note: ['a', 'c']}
{x: {x_1: 92}, note: ['b', 'd']}
...
```
to get the number of unique values from the first position of the json-array 'note' for topics 'A' or 'B',
where 10 <= 'time' <= 5000 and data.x.x_1 > 10:

```python
>>> citros = da.CitrosDB()
>>> citros.set_filter({'topic': ['A', 'B'], 'data.x.x_1': {'gt': 10}})\
...        .time(start = 10, end = 5000)\
...        .get_unique_counts('data.note[0]')
[(2,)]
```


To perform under the same conditions, but to get values grouped by topics:

```python
>>> citros.set_filter({'topic': ['A', 'B'], 'data.x.x_1': {'gt': 10}})\
...        .time(start = 10, end = 5000)\
...        .get_unique_counts('data.note[0]', group_by = ['topic'])
[('A', 2), ('B', 2)]
```


The same, but passing all constraintes by **filter_by** parameter:

```python
>>> citros.get_unique_counts('data.note[0]',
...                           group_by = ['topic'],
...                           filter_by = {'topic': ['A', 'B'], 
...                                        'time': {'gte': 10, 'lte': 5000}, 
...                                        'data.x.x_1' : {'gt':10}})
[('A', 2), ('B', 2)]
```

</details>


    
### Method `get_unique_values` {#citros_data_analysis.data_access.CitrosDB.get_unique_values}




```python
def get_unique_values(
    self,
    column_names,
    filter_by=None
)
```


<details>
  <summary>Description</summary>

Return unique values of the columns **column_names**.

---
#### Parameters

**```column_names```** :&ensp;**list**
:   Columns for which the unique combinations of the values will be found.


**```filter_by```** :&ensp;**dict**, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by **topic()**, **rid()**, **sid()**, **time()** and **set_filter()** and will override them.

---
#### Returns

&ensp;**list** or **list** of **tuples**
:   Each tuple contains unique combinations of the values for **column_names**.

---
#### Examples

Get unique values of type for topics 'A' or 'B', where 10 <= 'time' <= 5000 and data.x.x_1 > 10:

```python
>>> citros = da.CitrosDB()
>>> result = citros.set_filter({'topic': ['A', 'B'], 'data.x.x_1': {'gt': 10}})\
...                .time(start = 10, end = 5000)\
...                .get_unique_values(['type'])
>>> print(result)
['a', 'b']
```


The same, but passing all constraintes by **filter_by** parameter:

```python
>>> result = citros.get_unique_values(['type'], filter_by = {'topic': ['A', 'B'], 
...                                       'time': {'gte': 10, 'lte': 5000}, 
...                                       'data.x.x_1': {'gt':10}})
>>> print(result)
['a', 'b']
```

</details>


    
### Method `info` {#citros_data_analysis.data_access.CitrosDB.info}




```python
def info(
    self
)
```


<details>
  <summary>Description</summary>

Return info about the batch, selected according the previous calls of topic(), rid(), sid() and time() functions.

The output is a dictionary, that contains:
```python
'size': size of the selected data,
'sid_count': number of sids,
'sid_list': list of the sids,
'topic_count': number of topics,
'topic_list': list of topics,
'message_count': number of messages
```
If specific sid is set, also appends dictionary 'sids', with the following structure:
```python
'sids': {
  int: {
    'topics': {
      str: {
        'message_count': number of messages,
        'start_time': time when simulation started,
        'end_time': time when simulation ended,
        'duration': duration of the simalation process,
        'frequency': frequency of the simulation process (in Hz)}}}}
```
If topic is specified, appends dictionary 'topics':
```python
'topics': {
  str: {
    'type': type,
    'data_structure': structure of the data,
    'message_count': number of messages}}
```

---
#### Returns

&ensp;**[CitrosDict](#citros_data_analysis.data_access.CitrosDict "citros_data_analysis.data_access.CitrosDict")**
:   Information about the batch.

---
#### Examples

```python
>>> citros = da.CitrosDB()
>>> citros.info().print()
{
 'size': '27 kB',
 'sid_count': 3,
 'sid_list': [1, 2, 3],
 'topic_count': 4,
 'topic_list': ['A', 'B', 'C', 'D'],
 'message_count': 100
}
```


```python
>>> citros.topic('C').info().print()
{
 'size': '6576 bytes',
 'sid_count': 3,
 'sid_list': [1, 2, 3],
 'topic_count': 1,
 'topic_list': ['C'],
 'message_count': 24,
 'topics': {
   'C': {
     'type': 'c',
     'data_structure': {
       'data': {
         'x': {
           'x_1': 'int', 
           'x_2': 'float',
           'x_3': 'float'
         },
         'note': 'list',
         'time': 'float',
         'height': 'float'
       }
     },
     'message_count': 24
   }
 }
}
```


```python
>>> citros.sid([1,2]).info()
{
 'size': '20 kB',
 'sid_count': 2,
 'sid_list': [1, 2],
 'topic_count': 4,
 'topic_list': ['A', 'B', 'C', 'D'],
 'message_count': 76,
 'sids': {
   1: {
     'topics': {
       'A': {
          'message_count': 4,
          'start_time': 2000000000,
          'end_time': 17000000000,
          'duration': 15000000000,
          'frequency': 0.067
       },
       'B': {
          'message_count': 9,
...
          'duration': 150000000,
          'frequency': 0.067
       }
     }
   }
 }
}
```


```python
>>> citros.topic('C').sid(2).info()
{
 'size': '2192 bytes',
 'sid_count': 1,
 'sid_list': [2],
 'topic_count': 1,
 'topic_list': ['C'],
 'message_count': 8,
 'sids': {
   2: {
     'topics': {
       'C': {
         'message_count': 8,
         'start_time': 7000000170,
         'end_time': 19000000800,
         'duration': 12000000630,
         'frequency': 0.083
       }
     }
   }
 },
 'topics': {
   'C': {
     'type': 'c',
     'data_structure': {
       'data': {
         'x': {
           'x_1': 'int', 
           'x_2': 'float',
           'x_3': 'float'
         },
         'note': 'list',
         'time': 'float',
         'height': 'float'
         }
       },
     'message_count': 8
   }
 }
}
```

</details>


    
### Method `move_avg` {#citros_data_analysis.data_access.CitrosDB.move_avg}




```python
def move_avg(
    self,
    n_avg=None,
    n_skip=1
)
```


<details>
  <summary>Description</summary>

Compute moving average over **n_avg** massages and select each **n_skip**-th one.

Messages with different sids are processed separetly.
The value in 'rid' column is set as a minimum value among the 'rid' values of the averaged rows.

---
#### Parameters

**```n_avg```** :&ensp;**int**, optional
:   Number of messages to average.


**```n_skip```** :&ensp;**int**, default **1**
:   Number of the messages to skip.
    For example, if **skip** = 3, the 1th, the 4th, the 7th ... messages will be selected

---
#### Returns

&ensp;**[CitrosDB](#citros_data_analysis.data_access.CitrosDB "citros_data_analysis.data_access.CitrosDB")**
:   CitrosDB with parameters set for sampling method 'move_avg'.

---
#### Examples

Calculate moving average over each 5 messages of the topic 'A' 
and select every second row of the result:

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').move_avg(5,2).data()
```

</details>


    
### Method `multiple_y_plot` {#citros_data_analysis.data_access.CitrosDB.multiple_y_plot}




```python
def multiple_y_plot(
    self,
    df,
    x_label,
    y_labels,
    *args,
    fig=None,
    legend=True,
    title=None,
    **kwargs
)
```


<details>
  <summary>Description</summary>

Plot a series of vertically arranged graphs 'y vs. **x_label**', with the y-axis labels 
specified in the **y_labels** parameter.

Different colors correspond to different sids.

---
#### Parameters

**```df```** :&ensp;**pandas.DataFrame**
:   Data table.


**```x_label```** :&ensp;**str**
:   Label of the column to plot along x-axis.


**```y_labels```** :&ensp;**list** of **str**
:   Labels of the columns to plot along y-axis.


**```*args```** :&ensp;**Any**
:   Additional arguments to style lines, set color, etc, 
    see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)


**```fig```** :&ensp;**matplotlib.figure.Figure**, optional
:   If None, a new Figure will be created.


**```legend```** :&ensp;**bool**, default **True**
:   If True, show the legend with sids.


**```title```** :&ensp;**str**
:   Set title of the plot.

---
#### Returns

**```fig```** :&ensp;**matplotlib.figure.Figure**
:   &nbsp;


**```ax```** :&ensp;**list** of **matplotlib.axes.Axes**
:   &nbsp;

---
#### Other Parameters

**```**kwargs```**
:   Other keyword arguments, see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)

---
#### Examples

For topic 'A' from json-data column download 'data.x.x_1', 'data.x.x_2' and 'data.x.x_3' and 'data.time' columns:

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').data(['data.x.x_1', 'data.x.x_2', 'data.x.x_3', 'data.time'])
```


Plot three graphs: 'data.x.x_1' vs. 'data.time', 'data.x.x_2' vs. 'data.time' and 'data.x.x_3' vs. 'data.time':

```python
>>> fig, ax = citros.multiple_y_plot(df, 'data.time', ['data.x.x_1', 'data.x.x_2', 'data.x.x_3'])
```


Plot scatter graph:

```python
>>> fig, ax = citros.multiple_y_plot(df, 'data.time', ['data.x.x_1', 'data.x.x_2', 'data.x.x_3'], '.')
```

</details>


    
### Method `multiplot` {#citros_data_analysis.data_access.CitrosDB.multiplot}




```python
def multiplot(
    self,
    df,
    labels,
    *args,
    scale=True,
    fig=None,
    legend=True,
    title=None,
    **kwargs
)
```


<details>
  <summary>Description</summary>

Plot a matrix of N x N graphs, each displaying either the histogram with values distribution (for graphs on the diogonal) or
the relationship between variables listed in **labels**, with N being the length of **labels** list.

For non-diagonal graphs, colors are assigned to points according to sids.

---
#### Parameters

**```df```** :&ensp;**pandas.DataFrame**
:   Data table.


**```labels```** :&ensp;**list** of **str**
:   Labels of the columns to plot.


**```*args```** :&ensp;**Any**
:   Additional arguments to style lines, set color, etc, 
    see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)


**```scale```** :&ensp;**bool**, default **True**
:   Specify whether the axis range should be the same for x and y axes.


**```fig```** :&ensp;**matplotlib.figure.Figure**, optional
:   If None, a new Figure will be created.


**```legend```** :&ensp;**bool**, default **True**
:   If True, show the legend with sids.


**```title```** :&ensp;**str**
:   Set title of the plot.

---
#### Returns

**```fig```** :&ensp;**matplotlib.figure.Figure**
:   &nbsp;


**```ax```** :&ensp;**numpy.ndarray** of **matplotlib.axes.Axes**
:   &nbsp;

---
#### Other Parameters

**```**kwargs```**
:   Other keyword arguments, see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)

---
#### Examples

For topic 'A' from json-data column download 'data.x.x_1', 'data.x.x_2' and 'data.x.x_3':

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').data(['data.x.x_1', 'data.x.x_2', 'data.x.x_3'])
```


Plot nine graphs: histograms for three graphs on the diogonal, that represent 
distribution of the 'data.x.x_1', 'data.x.x_2' and 'data.x.x_3' values, and six graphs that show 
correlation between them; plot by dots and scale x and y axes ranges to oneintreval for each graph:

```python
>>> fig, ax = citros.multiplot(df, ['data.x.x_1'], '.' ,fig = fig, scale = True)
```

</details>


    
### Method `plot_3dgraph` {#citros_data_analysis.data_access.CitrosDB.plot_3dgraph}




```python
def plot_3dgraph(
    self,
    df,
    x_label,
    y_label,
    z_label,
    *args,
    ax=None,
    scale=True,
    legend=True,
    title=None,
    **kwargs
)
```


<details>
  <summary>Description</summary>

Plot 3D graph '**z_label** vs. **x_label** and **y_label**' for each sid, where **x_label**, **y_label** and **z_label**
are the labels of columns of the pandas.DataFrame **df**.

---
#### Parameters

**```df```** :&ensp;**pandas.DataFrame**
:   Data table.


**```x_label```** :&ensp;**str**
:   Label of the column to plot along x-axis.


**```y_label```** :&ensp;**str**
:   Label of the column to plot along y-axis.


**```*args```** :&ensp;**Any**
:   Additional arguments to style lines, set color, etc, 
    see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)


**```ax```** :&ensp;**matplotlib.axes.Axes**
:   Figure axis to plot on. If not specified, the new pair of fig, ax will be created.


**```scale```** :&ensp;**bool**, default **True**
:   Specify whether the axis range should be the same for all axes.


**```legend```** :&ensp;**bool**, default **True**
:   If True, show the legend with sids.


**```title```** :&ensp;**str**
:   Set title of the plot.

---
#### Returns

**```fig```** :&ensp;**matplotlib.figure.Figure**
:   Created figure if **ax** was not passed.


**```ax```** :&ensp;**matplotlib.axes.Axes**
:   Created ax if **ax** is not passed.

---
#### Other Parameters

**```**kwargs```**
:   Other keyword arguments, see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)

---
#### Examples

Import matplotlib and mplot3d for 3D plots and create figure to plot on:

```python
>>> import matplotlib.pyplot as plt
>>> from mpl_toolkits import mplot3d
>>> fig = plt.Figure(figsize=(6, 6))
>>> ax = fig.add_subplot(111, projection = '3d')
```


For topic 'A' from json-data column download 'data.x.x_1', 'data.x.x_2' and 'data.x.x_3' columns:

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').data(['data.x.x_1', 'data.x.x_2', 'data.x.x_3'])
```


Make 3D plot with dashed lines; **scale** = True aligns all axes to have the same range:

```python
>>> citros.plot_3dgraph(df, 'data.x.x_1', 'data.x.x_2', 'data.x.x_1', '--', ax = ax, scale = True)
```

</details>


    
### Method `plot_graph` {#citros_data_analysis.data_access.CitrosDB.plot_graph}




```python
def plot_graph(
    self,
    df,
    x_label,
    y_label,
    *args,
    ax=None,
    legend=True,
    title=None,
    **kwargs
)
```


<details>
  <summary>Description</summary>

Plot graph '**y_label** vs. **x_label**' for each sid, where **x_label** and **y_label**
are the labels of columns of the pandas.DataFrame **df**.

---
#### Parameters

**```df```** :&ensp;**pandas.DataFrame**
:   Data table.


**```x_label```** :&ensp;**str**
:   Label of the column to plot along x-axis.


**```y_label```** :&ensp;**str**
:   Label of the column to plot along y-axis.


**```*args```** :&ensp;**Any**
:   Additional arguments to style lines, set color, etc, 
    see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)


**```ax```** :&ensp;**matplotlib.axes.Axes**
:   Figure axis to plot on. If not specified, the new pair of fig, ax will be created.


**```legend```** :&ensp;**bool**, default **True**
:   If True, show the legend with sids.


**```title```** :&ensp;**str**
:   Set title of the plot.

---
#### Returns

**```fig```** :&ensp;**matplotlib.figure.Figure**
:   Created figure if **ax** was not passed.


**```ax```** :&ensp;**matplotlib.axes.Axes**
:   Created ax if **ax** is not passed.

---
#### Other Parameters

**```**kwargs```**
:   Other keyword arguments, see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)

---
#### Examples

Import matplotlib and create figure to plot on:

```python
>>> import matplotlib.pyplot as plt
>>> fig, ax = plt.subplots()
```


From topic 'A' from json-data column download 'data.x.x_1' and 'data.x.x_2' columns:

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').data(['data.x.x_1', 'data.x.x_2'])
```


Plot **data.x.x_1** vs. **data.x.x_2**:

```python
>>> citros.plot_graph(df, 'data.x.x_1', 'data.x.x_2', ax = ax)
```


Generate a new figure and plot the previous graph but using a dotted line:

```python
>>> import matplotlib.pyplot as plt
>>> fig, ax = plt.subplots()
>>> citros.plot_graph(df, 'data.x.x_1', 'data.x.x_2', '.', ax = ax)
```

</details>


    
### Method `rid` {#citros_data_analysis.data_access.CitrosDB.rid}




```python
def rid(
    self,
    start=0,
    end=None,
    count=None
)
```


<details>
  <summary>Description</summary>

Set constraints on rid.

---
#### Parameters

**```start```** :&ensp;**int**, default **0**
:   The lower limit for rid values.


**```end```** :&ensp;**int**, optional
:   The higher limit for rid, the end is included.


**```count```** :&ensp;**int**, optional
:   Used only if the **end** is not set.
    Number of rid to return in the query, starting form the **start**.

---
#### Returns

&ensp;**[CitrosDB](#citros_data_analysis.data_access.CitrosDB "citros_data_analysis.data_access.CitrosDB")**
:   CitrosDB with set 'rid' parameter.

---
#### Examples

Get data for for topic 'A' where rid is in the range of 0 <= rid <= 9 :

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').rid(start = 0, end = 9).data()
```


or the same with **count**:

```python
>>> df = citros.topic('A').rid(start = 0, count = 10).data()
```


For rid >= 5:

```python
>>> df = citros.topic('A').rid(start = 5).data()
```

</details>


    
### Method `set_filter` {#citros_data_analysis.data_access.CitrosDB.set_filter}




```python
def set_filter(
    self,
    filter_by=None
)
```


<details>
  <summary>Description</summary>

Set constraints on query.

Allows to set constraints on json-data columns.

---
#### Parameters

**```filter_by```** :&ensp;**dict**
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by **topic()**, **rid()**, **sid()** and **time()** and will override them.
    If sampling method is used, constraints on additional columns are applied BEFORE sampling while
    constraints on columns from json-data are applied AFTER sampling.

---
#### Returns

&ensp;**[CitrosDB](#citros_data_analysis.data_access.CitrosDB "citros_data_analysis.data_access.CitrosDB")**
:   CitrosDB with set constraints.

---
#### See Also

`topic() : set topic name to query`
`sid() : set sid values to query`
`rid() : set rid values to query`
`time() : set time constraints`

---
#### Examples

if the structure of the data column is the following:

```python
{x: {x_1: 1}, note: [11, 34]}
{x: {x_1: 2}, note: [11, 35]}
...
```
to get data for topic 'A' where values of json-data column 10 < data.x.x_1 <= 20:

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').set_filter({'data.x.x_1': {'gt': 10, 'lte': 20}}).data()
```


get data where the value on the first position in the json-array 'note' equals 11 or 12:

```python
>>> df = citros.topic('A').set_filter({'data.note[0]': [11, 12]}).data()
```

</details>


    
### Method `set_order` {#citros_data_analysis.data_access.CitrosDB.set_order}




```python
def set_order(
    self,
    order_by=None
)
```


<details>
  <summary>Description</summary>

Apply sorting to the result of the query.

Sort the result of the query in ascending or descending order.

---
#### Parameters

**```order_by```** :&ensp;**dict**, optional
:   Keys of the dictionary must match labels of the columns,
    values - define ascending ('asc') or descending ('desc') order.

---
#### Examples

Get data for topic 'A' and sort the result by sid in ascending order and by rid in descending order.

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').set_order({'sid': 'asc', 'rid': 'desc'}).data()
```

</details>


    
### Method `sid` {#citros_data_analysis.data_access.CitrosDB.sid}




```python
def sid(
    self,
    value=None
)
```


<details>
  <summary>Description</summary>

Set constraints on sid.

---
#### Parameters

**```value```** :&ensp;**int** or **list** of **ints**, optional
:   Limits for sid.
    If nothing is passed, then the default value of sid is used (ENV parameter "CITROS_SIMULATION_RUN_ID").
    If the default value does not exist, no limits for sid are applied.

---
#### Returns

&ensp;**[CitrosDB](#citros_data_analysis.data_access.CitrosDB "citros_data_analysis.data_access.CitrosDB")**
:   CitrosDB with set 'sid' parameter.

---
#### Examples

Get data for topic 'A' where sid values are 1 or 2:

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').sid([1,2]).data()
```

</details>


    
### Method `skip` {#citros_data_analysis.data_access.CitrosDB.skip}




```python
def skip(
    self,
    n_skip=None
)
```


<details>
  <summary>Description</summary>

Select each **n_skip**-th message.

Messages with different sids are selected separetly.

---
#### Parameters

**```skip```** :&ensp;**int**, optional
:   Control number of the messages to skip.

---
#### Returns

&ensp;**[CitrosDB](#citros_data_analysis.data_access.CitrosDB "citros_data_analysis.data_access.CitrosDB")**
:   CitrosDB with parameters set for sampling method 'skip'.

---
#### Examples

To get every 3th message of the topic 'A':

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').skip(3).data()
```


the 1th, the 4th, the 7th ... messages will be selected
</details>


    
### Method `time` {#citros_data_analysis.data_access.CitrosDB.time}




```python
def time(
    self,
    start=0,
    end=None,
    duration=None
)
```


<details>
  <summary>Description</summary>

Set constraints on time.

---
#### Parameters

**```start```** :&ensp;**int**, default **0**
:   The lower limit for time values.


**```end```** :&ensp;**int**, optional
:   The higher limit for time, the end is included.


**```duration```** :&ensp;**int**, optional
:   Used only if the **end** is not set.
    Time interval to return in the query, starting form the **start**.

---
#### Returns

&ensp;**[CitrosDB](#citros_data_analysis.data_access.CitrosDB "citros_data_analysis.data_access.CitrosDB")**
:   CitrosDB with set 'time' parameter.

---
#### Examples

Get data for for topic 'A' where time is in the range 10ns <= time <= 20ns:

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').time(start = 10, end = 20).data()
```


To set time range 'first 10ns starting from 10th nanosecond', that means 10ns <= time < 20ns:

```python
>>> df = citros.topic('A').time(start = 10, duration = 10).data()
```


For time >= 20:

```python
>>> df = citros.topic('A').time(start = 20).data()
```

</details>


    
### Method `time_plot` {#citros_data_analysis.data_access.CitrosDB.time_plot}




```python
def time_plot(
    self,
    ax,
    *args,
    topic_name=None,
    var_name=None,
    time_step=1.0,
    sids=None,
    y_label=None,
    title_text=None,
    legend=True,
    **kwargs
)
```


<details>
  <summary>Description</summary>

Plot **var_name** vs. **Time** for each of the sids, where **Time** = **time_step** * rid.

---
#### Parameters

**```ax```** :&ensp;**matplotlib.axes.Axes**
:   Figure axis to plot on.


**```*args```** :&ensp;**Any**
:   Additional arguments to style lines, set color, etc, 
    see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)


**```topic_name```** :&ensp;**str**
:   Input topic name. If specified, will override value that was set by **topic()** method.


**```var_name```** :&ensp;**str**
:   Name of the variable to plot along y-axis.


**```time_step```** :&ensp;**float** or **int**, default **1.0**
:   Time step, **Time** = **time_step** * rid.


**```sids```** :&ensp;**list**
:   List of the sids. If specified, will override values that were set by **sid()** method.
    If not specified, data for all sids is used.


**```y_label```** :&ensp;**str**
:   Label to set to y-axis. Default **var_name**.


**```title_text```** :&ensp;**str**
:   Title of the figure. Default '**var_y_name** vs. Time'.


**```legend```** :&ensp;**bool**
:   If True, show the legend with sids.

---
#### Other Parameters

**```**kwargs```**
:   Other keyword arguments, see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)

---
#### Examples

Import matplotlib and create figure to plot on:

```python
>>> import matplotlib.pyplot as plt
>>> fig, ax = plt.subplots()
```


For topic 'A' plot **data.x.x_1** vs. **Time** for all existing sids, **Time** = 0.5 * rid

```python
>>> citros = da.CitrosDB()
>>> citros.time_plot(ax, topic_name = 'A', var_name = 'data.x.x_1', time_step = 0.5)
```


It is possible to set topic by **topic()** method:

```python
>>> citros.topic('A').time_plot(ax, var_name = 'data.x.x_1', time_step = 0.5)
```


Create a new figure and plot only part of the data, where 'data.x.x_1' <= 0; plot by dashed line:

```python
>>> fig, ax = plt.subplots()
>>> citros.topic('A').set_filter({'data.x.x_1':{'lte': 0}})\
          .time_plot(ax, '--', var_name = 'data.x.x_1', time_step = 0.5)
```

</details>


    
### Method `topic` {#citros_data_analysis.data_access.CitrosDB.topic}




```python
def topic(
    self,
    topic_name=None
)
```


<details>
  <summary>Description</summary>

Select topic.

---
#### Parameters

**```topic_name```** :&ensp;**str**
:   Name of the topic.

---
#### Returns

&ensp;**[CitrosDB](#citros_data_analysis.data_access.CitrosDB "citros_data_analysis.data_access.CitrosDB")**
:   CitrosDB with set 'topic' parameter.

---
#### Examples

Get data for topic name 'A':

```python
>>> citros = da.CitrosDB()
>>> df = citros.topic('A').data()
```

</details>


    
### Method `xy_plot` {#citros_data_analysis.data_access.CitrosDB.xy_plot}




```python
def xy_plot(
    self,
    ax,
    *args,
    topic_name=None,
    var_x_name=None,
    var_y_name=None,
    sids=None,
    x_label=None,
    y_label=None,
    title_text=None,
    legend=True,
    **kwargs
)
```


<details>
  <summary>Description</summary>

Plot **var_y_name** vs. **var_x_name** for each of the sids.

---
#### Parameters

**```ax```** :&ensp;**matplotlib.axes.Axes**
:   Figure axis to plot on.


**```*args```** :&ensp;**Any**
:   Additional arguments to style lines, set color, etc, 
    see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)


**```topic_name```** :&ensp;**str**
:   Input topic name. If specified, will override value that was set by **topic()** method.


**```var_x_name```** :&ensp;**str**
:   Name of the variable to plot along x-axis.


**```var_y_name```** :&ensp;**str**
:   Name of the variable to plot along y-axis.


**```sids```** :&ensp;**list**, optional
:   List of the sids. If specified, will override values that were set by **sid()** method.
    If not specified, data for all sids is used.


**```x_label```** :&ensp;**str**, optional
:   Label to set to x-axis. Default **var_x_name**.


**```y_label```** :&ensp;**str**, optional
:   Label to set to y-axis. Default **var_y_name**.


**```title_text```** :&ensp;**str**, optional
:   Title of the figure. Default '**var_y_name** vs. **var_x_name**'.


**```legend```** :&ensp;**bool**
:   If True, show the legend with sids.

---
#### Other Parameters

**```**kwargs```**
:   Other keyword arguments, see [matplotlib.axes.Axes.plot](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.plot.html)

---
#### Examples

```python
>>> import matplotlib.pyplot as plt
>>> fig, ax = plt.subplots()
```


For topic 'A' plot 'data.x.x_1' vs. 'data.x.x_2' for all existing sids:

```python
>>> citros = da.CitrosDB()
>>> citros.xy_plot(ax, topic_name = 'A', var_x_name = 'data.x.x_1', var_y_name = 'data.x.x_2')
```


It is possible to set topic by **topic()** method:

```python
>>> citros.topic('A').xy_plot(ax, var_x_name = 'data.x.x_1', var_y_name = 'data.x.x_2')
```


Create new figure and plot only part of the data, where 'data.x.x_1' <= 0, sid = 1 and 2; plot by dashed lines:

```python
>>> fig, ax = plt.subplots()
>>> citros.topic('A').set_filter({'data.x.x_1':{'lte': 0}}).sid([1,2])\
          .xy_plot(ax, '--', var_x_name = 'data.x.x_1', var_y_name = 'data.x.x_2')
```

</details>


    
## Class `CitrosDict` {#citros_data_analysis.data_access.CitrosDict}





```python
class CitrosDict(
    *args,
    **kwargs
)
```


<details>
  <summary>Description</summary>

Citros dictionary

CitrosDict is a dictionary-like object, that allows to print the content as a json-object.



    
#### Ancestors

* [builtins.dict](#builtins.dict)

</details>






    
### Method `print` {#citros_data_analysis.data_access.CitrosDict.print}




```python
def print(
    self
)
```


<details>
  <summary>Description</summary>

Print content of the CitrosDict object in a 'json'-style.
</details>


    
### Method `to_json` {#citros_data_analysis.data_access.CitrosDict.to_json}




```python
def to_json(
    self
)
```


<details>
  <summary>Description</summary>

Convert to json string.

---
#### Returns

&ensp;**str**
:   json_str


</details>