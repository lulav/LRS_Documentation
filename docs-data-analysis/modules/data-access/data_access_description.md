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

#### Parameters

**```host```** :&ensp;<code>str</code>
:   Database host adress.
    If None, uses predefined ENV parameter "PG_HOST".


**```user```** :&ensp;<code>str</code>
:   User name.
    If not specified, uses predefined ENV parameter "PG_USER".


**```password```** :&ensp;<code>str</code>
:   Password.
    If not specified, uses predefined ENV parameter "PG_PASSWORD".


**```database```** :&ensp;<code>str</code>
:   Database name.
    If not specified, uses predefined ENV parameter "PG_DATABASE".


**```schema```** :&ensp;<code>str</code>
:   If None, uses predefined ENV parameter "PG_SCHEMA".


**```batch```** :&ensp;<code>str</code>
:   Batch name.
    If not specified, uses predefined ENV parameter "bid".


**```port```** :&ensp;<code>str</code>
:   If None, uses predefined ENV parameter "PG_PORT".


**```sid```** :&ensp;<code>int</code>, optional
:   Default sid.
    If not specified, uses predefined ENV parameter "CITROS_SIMULATION_RUN_ID" if it exists or None.


**```debug_flag```** :&ensp;<code>bool</code>, default <code>False</code>
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

Average <code>n\_avg</code> number of messages.

Messages with different sids are processed separetly. 
The value in 'rid' column is set as a minimum value among the 'rid' values of the averaged rows.

#### Parameters

**```n_avg```** :&ensp;<code>int</code>
:   Number of messages to average.

#### Returns

&ensp;<code>[CitrosDB](#citros\_data\_analysis.data\_access.CitrosDB "citros\_data\_analysis.data\_access.CitrosDB")</code>
:   CitrosDB with parameters set for sampling method 'avg'.

#### Examples

To average each 3 messages of the topic 'A' and get the result:

```python
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

#### Parameters

**```data_names```** :&ensp;<code>list</code>, optional
:   Labels of the columns from json data column.

#### Returns

&ensp;<code>pandas.DataFrame</code>
:   Table with selected data.

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

Return sizes of the all tables in the current schema.

#### Returns

&ensp;<code>list</code> of <code>tuples</code>
:   Each tuple contains name of the table, table size and total size with indexes.


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

Return number of the rows in the column <code>column\_name</code>.

#### Parameters

**```column_name```** :&ensp;<code>str</code>
:   Label of the column.


**```group_by```** :&ensp;<code>list</code>, optional
:   Labels of the columns to group by. If blank, do not group.


**```filter_by```** :&ensp;<code>dict</code>, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by <code>topic()</code>, <code>rid()</code>, <code>sid()</code>, <code>time()</code> and <code>set\_filter()</code> and will override them.

#### Returns

&ensp;<code>list</code> of <code>tuples</code> or <code>None</code>
:   Number of rows in <code>column\_name</code>.

#### Examples

If the structure of the data column is the following:

```python
{x: {x_1: 52}, note: ['b', 'e']}
{x: {x_1: 11}, note: ['a', 'c']}
{x: {x_1: 92}, note: ['b', 'd']}
...
```
to get the number of values from the first position of the json-array 'note' for topics 'A' or 'B',
where 10 < 'time' <= 5000 and data.x.x_1 >= 10:

```python
>>> citros.get_counts('data.note[0]',
...                    filter_by = {'topic': ['A', 'B'], 
...                                 'time': {'gt': 10, 'lte': 5000}, 
...                                 'data.x.x_1' : {'gte':10}})
[(30,)]
```


To perform under the same conditions, but to get values grouped by topics:

```python
>>> citros.get_counts('data.note[0]',
...                    group_by = ['topic'],
...                    filter_by = {'topic': ['A', 'B'], 
...                                 'time': {'gt': 10, 'lte': 5000}, 
...                                 'data.x.x_1' : {'gte':10}})
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

Return structure of the "data".

Each tuple conatains topic and type names, structure of the corresponding data and number of sids.

#### Parameters

**```topic```** :&ensp;<code>list</code> or <code>list</code> of <code>str</code>, optional
:   list of the topics to show data structure for.
    Have higher priority, than those defined by <code>topic()</code> and <code>set\_filter()</code> methods 
    and will override them.
    If not specified, shows data structure for all topics.

#### Returns

&ensp;<code>list</code> of <code>tuples</code>
:   Each tuple conatains topic and type names and structure of the corresponding data.


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

Return maximum value of the column <code>column\_name</code>.

#### Parameters

**```column_name```** :&ensp;<code>str</code>
:   Label of the column.


**```filter_by```** :&ensp;<code>dict</code>, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by <code>topic()</code>, <code>rid()</code>, <code>sid()</code>, <code>time()</code> and <code>set\_filter()</code> and will override them.

#### Returns

&ensp;<code>int, float, str</code> or <code>None</code>
:   Maximum value of the column <code>column\_name</code>.

#### Examples

To get max value of the column 'rid' where topics are 'A' or 'B', 10 < 'time' <= 5000 and data.x.x_1 >= 10:

```python
>>> result = citros.get_max_value('rid',
...                               filter_by = {'topic': ['A', 'B'], 
...                                            'time': {'gt': 10, 'lte': 5000}, 
...                                            'data.x.x_1' : {'gte':10}})
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

Return minimum value of the column <code>column\_name</code>.

#### Parameters

**```column_name```** :&ensp;<code>str</code>
:   Label of the column.


**```filter_by```** :&ensp;<code>dict</code>, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by <code>topic()</code>, <code>rid()</code>, <code>sid()</code>, <code>time()</code> and <code>set\_filter()</code> and will override them.

#### Returns

&ensp;<code>int, float, str</code> or <code>None</code>
:   Minimum value of the column <code>column\_name</code>.

#### Examples

To get min value of the column 'rid' where topics are 'A' or 'B', 10 < 'time' <= 5000 and data.x.x_1 >= 10:

```python
>>> result = citros.get_min_value('rid',
...                               filter_by = {'topic': ['A', 'B'], 
...                                            'time': {'gt': 10, 'lte': 5000}, 
...                                            'data.x.x_1' : {'gte':10}})
```

</details>


    
### Method `get_sid_tables` {#citros_data_analysis.data_access.CitrosDB.get_sid_tables}




```python
def get_sid_tables(
    self,
    topic=None,
    data_query=None,
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

#### Parameters

**```topic```** :&ensp;<code>str</code>
:   Name of the topic.
    Have higher priority than defined by <code>topic()</code>.
    May be overrided by <code>filter\_by</code> argument.


**```data_query```** :&ensp;<code>list</code>, optional
:   Labels of the data to download from the json-format column "data".
    If blank list, then all columns are are downloaded.


**```additional_columns```** :&ensp;<code>list</code>, optional
:   Columns to download outside the json "data".
    If blank list, then all columns are are downloaded.


**```filter_by```** :&ensp;<code>dict</code>, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by <code>topic()</code>, <code>rid()</code>, <code>sid()</code>, <code>time()</code> and <code>set\_filter()</code> and will override them.


**```order_by```** :&ensp;<code>dict</code>, optional
:   Keys must match labels of the columns, values specify ascending ('asc') or descending ('desc') order.
    Conditions, passed here, have higher priority over those defined by <code>set\_order()</code> and will override them.


**```method```** :&ensp;`{'', 'avg', 'move_avg', 'skip'}`, optional
:   Method of sampling:
    'avg' - average - average <code>n\_avg</code> rows;
    'move_avg' - moving average - average <code>n\_avg</code> rows and return every <code>n\_skip</code>-th row;
    'skip' - skiping <code>n\_skip</code> rows;
    '' - no sampling.
    If not specified, no sampling is applied


**```n_avg```** :&ensp;<code>int</code>
:   Used only if <code>method</code> is 'move_avg' or 'avg'.
    Number of rows for averaging.


**```n_skip```** :&ensp;<code>int</code>
:   Used only if <code>method</code> is 'move_avg' or 'skip'.
    Number of rows to skip in a result output. 
    For example, if skip = 2, only every second row will be returned.

#### Returns

&ensp;<code>dict</code> of <code>pandas.DataFrames</code>
:   dict with tables, key is a value of sid.

#### Examples

Download averaged data for each sid separetly:

```python
>>> dfs = citros.get_sid_tables('A', 
...                             data_query = ['data.x.x_1'],
...                             additional_columns = [], 
...                             filter_by = {}, 
...                             order_by = {'rid': 'asc'}, 
...                             method = 'avg', 
...                             n_avg = 2)
```


```python
>>> print('sid values are: {}'.format(list(dfs.keys())))
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

Return number of the unique values in the column <code>column\_name</code>.

#### Parameters

**```column_name```** :&ensp;<code>str</code>
:   Column to count its unique values.


**```group_by```** :&ensp;<code>list</code>, optional
:   Labels of the columns to group by. If blank, do not group.


**```filter_by```** :&ensp;<code>dict</code>, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by <code>topic()</code>, <code>rid()</code>, <code>sid()</code>, <code>time()</code> and <code>set\_filter()</code> and will override them.

#### Returns

&ensp;<code>int</code> or <code>list</code> or <code>None</code>
:   Counts of the unique values in <code>column\_name</code>.

#### Examples

If the structure of the data column is the following:

```python
{x: {x_1: 52}, note: ['b', 'e']}
{x: {x_1: 11}, note: ['a', 'c']}
{x: {x_1: 92}, note: ['b', 'd']}
...
```
to get the number of unique values from the first position of the json-array 'note' for topics 'A' or 'B',
where 10 < 'time' <= 5000 and data.x.x_1 >= 10:

```python
>>> citros.get_counts('data.note[0]',
...                    filter_by = {'topic': ['A', 'B'], 
...                                 'time': {'gt': 10, 'lte': 5000}, 
...                                 'data.x.x_1' : {'gte':10}})
[(2,)]
```


To perform under the same conditions, but to get values grouped by topics:

```python
>>> citros.get_counts('data.note[0]',
...                    group_by = ['topic'],
...                    filter_by = {'topic': ['A', 'B'], 
...                                 'time': {'gt': 10, 'lte': 5000}, 
...                                 'data.x.x_1' : {'gte':10}})
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

Return unique values of the columns <code>column\_names</code>.

#### Parameters

**```column_names```** :&ensp;<code>list</code>
:   Columns for which the unique combinations of the values will be found.


**```filter_by```** :&ensp;<code>dict</code>, optional
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by <code>topic()</code>, <code>rid()</code>, <code>sid()</code>, <code>time()</code> and <code>set\_filter()</code> and will override them.

#### Returns

&ensp;<code>list</code> or <code>list</code> of <code>tuples</code>
:   Each tuple contains unique combinations of the values for <code>column\_names</code>.

#### Examples

To get unique values of type for topics 'A' or 'B', where 10 < 'time' <= 5000 and data.x.x_1 >= 10:

```python
>>> result = citros.get_unique_values(['type'], filter_by = {'topic': ['A', 'B'], 
...                                       'time': {'gt': 10, 'lte': 5000}, 
...                                       'data.x.x_1': {'gte':10}})
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

Returns dictionary, that contains:
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
                'frequency': frequency of the simulation process}}}}
```
If topic is specified, appends dictionary 'topics':
```python
'topics': {
    str: {
        'type': type,
        'data_structure': structure of the data,
        'message_count': number of messages}}
```
            
#### Returns

&ensp;<code>[CitrosDict](#citros\_data\_analysis.data\_access.CitrosDict "citros\_data\_analysis.data\_access.CitrosDict")</code>
:   Information about the batch.

#### Examples

```python
>>> citros.info()
{'size': '27 kB',
'sid_count': 3,
'sid_list': [1, 2, 3],
'topic_count': 4,
'topic_list': ['A', 'B', 'C', 'D'],
'message_count': 100}
```


```python
>>> citros.topic('C').info()
{'size': '6576 bytes',
'sid_count': 3,
'sid_list': [1, 2, 3],
'topic_count': 1,
'topic_list': ['C'],
'message_count': 24,
'topics': 
    {'C': 
        {'type': 'c',
        'data_structure': 
            {'data':
                {'x': 
                    {'x_1': 'int', 
                    'x_2': 'float',
                    'x_3': 'float'},
                'note': 'list',
                'time': 'float',
                'height': 'float'}},
        'message_count': 24}}}
```


```python
>>> citros.sid([1,2]).info()
{'size': '20 kB',
'sid_count': 2,
'sid_list': [1, 2],
'topic_count': 4,
'topic_list': ['A', 'B', 'C', 'D'],
'message_count': 76,
'sids': 
    {1: 
        {'topics': 
            {'A': 
                {'message_count': 4,
                'start_time': 2.0,
                'end_time': 17.0,
                'duration': 15.0,
                'frequency': 0.067},
            'B': 
                {'message_count': 9,
...
                'duration': 15.0,
                'frequency': 0.0667}}}}}
```


```python
>>> citros.topic('C').sid(2).info()
{'size': '2192 bytes',
'sid_count': 1,
'sid_list': [2],
'topic_count': 1,
'topic_list': ['C'],
'message_count': 8,
'sids': 
    {2: 
        {'topics': 
            {'C': 
                {'message_count': 8,
                'start_time': 7.00000017,
                'end_time': 19.0000008,
                'duration': 12.00000063,
                'frequency': 0.08333332895833356}}}},
'topics': 
    {'C': 
        {'type': 'c',
        'data_structure':
            {'data': 
                {'x': 
                    {'x_1': 'int', 
                    'x_2': 'float',
                    'x_3': 'float'},
                'note': 'list',
                'time': 'float',
                'height': 'float'}},
'message_count': 8}}}
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

Compute moving average over <code>n\_avg</code> massages and select each <code>n\_skip</code>-th one.

Messages with different sids are processed separetly.
The value in 'rid' column is set as a minimum value among the 'rid' values of the averaged rows.

#### Parameters

**```n_avg```** :&ensp;<code>int</code>, optional
:   Number of messages to average.


**```n_skip```** :&ensp;<code>int</code>, default <code>1</code>
:   Number of the messages to skip.
    For example, if <code>skip</code> = 3, the 1th, the 4th, the 7th ... messages will be selected

#### Returns

&ensp;<code>[CitrosDB](#citros\_data\_analysis.data\_access.CitrosDB "citros\_data\_analysis.data\_access.CitrosDB")</code>
:   CitrosDB with parameters set for sampling method 'move_avg'.

#### Examples

To calculate moving average over each 5 messages of the topic 'A' 
and select every second row of the result:

```python
>>> df = citros.topic('A').move_avg(5,2).data()
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
    **kwargs
)
```


<details>
  <summary>Description</summary>

Plot graph '<code>y\_label</code> vs. <code>x\_label</code>' for each sid, where <code>x\_label</code> and <code>y\_label</code>
are the labels of columns of the table <code>df</code>.

#### Parameters

**```df```** :&ensp;<code>pandas.DataFrame</code>
:   Data table.


**```x_label```** :&ensp;<code>str</code>
:   Label of the column to plot along x-axis.


**```y_label```** :&ensp;<code>str</code>
:   Label of the column to plot along y-axis.

#### Other Parameters

**```*args```** :&ensp;<code>Any </code>
:   Other positional arguments, see matplotlib.axes.Axes.plot


**```**kwargs```** :&ensp;<code>dict</code>
:   Other keyword arguments, see matplotlib.axes.Axes.plot


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

#### Parameters

**```start```** :&ensp;<code>int</code>, default <code>0</code>
:   The lower limit for rid values.


**```end```** :&ensp;<code>int</code>, optional
:   The higher limit for rid, the end is included.


**```count```** :&ensp;<code>int</code>, optional
:   Used only if the <code>end</code> is not set.
    Number of rid to return in the query, starting form the <code>start</code>.

#### Returns

&ensp;<code>[CitrosDB](#citros\_data\_analysis.data\_access.CitrosDB "citros\_data\_analysis.data\_access.CitrosDB")</code>
:   CitrosDB with set 'rid' parameter.

#### Examples

To get data for for topic 'A' where rid is in the range of 0 <= rid <= 9 :

```python
>>> df = citros.topic('A').rid(start = 0, end = 9).data()
```


or the same with <code>count</code>:

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

#### Parameters

**```filter_by```** :&ensp;<code>dict</code>
:   Constraints to apply on columns: {key_1: value_1, key_2: value_2, ...}.
    key_n - must match labels of the columns, 
    value_n  - in the case of equality: list of exact values
               in the case of inequality: dict with "gt", "gte", "lt" & "lte" as keys for ">", ">=", "<" & "<=".
    Conditions, passed here, have higher priority over those defined by <code>topic()</code>, <code>rid()</code>, <code>sid()</code> and <code>time()</code> and will override them.
    If sampling method is used, constraints on additional columns are applied BEFORE sampling while
    constraints on columns from json-data are applied AFTER sampling.

#### Returns

&ensp;<code>[CitrosDB](#citros\_data\_analysis.data\_access.CitrosDB "citros\_data\_analysis.data\_access.CitrosDB")</code>
:   CitrosDB with set constraints.

#### See Also

<code>topic</code>
:   set topic name to query

<code>sid</code>
:   set sid values to query

<code>rid</code>
:   set rid values to query

<code>time</code>
:   set time constraints

#### Examples

if the structure of the data column is the following:

```python
{x: {x_1: 1}, note: [11, 34]}
{x: {x_1: 2}, note: [11, 35]}
...
```
to get data for topic 'A' where values of json-data column 10 < data.x.x_1 <= 20:

```python
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

#### Parameters

**```order_by```** :&ensp;<code>dict</code>, optional
:   Keys of the dictionary must match labels of the columns,
    values - define ascending ('asc') or descending ('desc') order.

#### Examples

Get data for topic 'A' and sort the result by sid in ascending order and by rid in descending order.
```python
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

#### Parameters

**```value```** :&ensp;<code>int</code> or <code>list</code> of <code>ints</code>, optional
:   Limits for sid.
    If nothing is passed, then the default value of sid is used (ENV parameter "CITROS_SIMULATION_RUN_ID").
    If the default value does not exist, no limits for sid are applied.

#### Returns

&ensp;<code>[CitrosDB](#citros\_data\_analysis.data\_access.CitrosDB "citros\_data\_analysis.data\_access.CitrosDB")</code>
:   CitrosDB with set 'sid' parameter.

#### Examples

To get data for topic 'A' where sid values are 1 or 2:

```python
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

Select each <code>n\_skip</code>-th message.

Messages with different sids are selected separetly.

#### Parameters

**```skip```** :&ensp;<code>int</code>, optional
:   Control number of the messages to skip.

#### Returns

&ensp;<code>[CitrosDB](#citros\_data\_analysis.data\_access.CitrosDB "citros\_data\_analysis.data\_access.CitrosDB")</code>
:   CitrosDB with parameters set for sampling method 'skip'.

#### Examples

To get every 3th message of the topic 'A':

```python
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

#### Parameters

**```start```** :&ensp;<code>int</code>, default <code>0</code>
:   The lower limit for time values.


**```end```** :&ensp;<code>int</code>, optional
:   The higher limit for time, the end is included.


**```duration```** :&ensp;<code>int</code>, optional
:   Used only if the <code>end</code> is not set.
    Time interval to return in the query, starting form the <code>start</code>.

#### Returns

&ensp;<code>[CitrosDB](#citros\_data\_analysis.data\_access.CitrosDB "citros\_data\_analysis.data\_access.CitrosDB")</code>
:   CitrosDB with set 'time' parameter.

#### Examples

To get data for for topic 'A' where time is in the range 10s <= time <= 20s:

```python
>>> df = citros.topic('A').time(start = 10, end = 20).data()
```


To set time range 'first 10s starting from 10th second', that means 10s <= time < 20s:

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
    topic_name=None,
    var_name=None,
    time_step=1.0,
    sids=None,
    y_label=None,
    title_text=None
)
```


<details>
  <summary>Description</summary>

Plot <code>var\_name</code> vs. <code>Time</code> for each of the sids, where <code>Time</code> = <code>time\_step</code> * rid.

#### Parameters

**```ax```** :&ensp;<code>matplotlib.axes.Axes</code>
:   Figure axis to plot on.


**```topic_name```** :&ensp;<code>str</code>
:   Input topic name. If specified, will override value that was set by <code>topic()</code> method.


**```var_name```** :&ensp;<code>str</code>
:   Name of the variable to plot along y-axis.


**```time_step```** :&ensp;<code>float</code> or <code>int</code>, default <code>1.0</code>
:   Time step, <code>Time</code> = <code>time\_step</code> * rid.


**```sids```** :&ensp;<code>list</code>
:   List of the sids. If specified, will override values that were set by <code>sid()</code> method.
    If not specified, data for all sids is used.


**```y_label```** :&ensp;<code>str</code>
:   Label to set to y-axis. Default <code>var\_name</code>.


**```title_text```** :&ensp;<code>str</code>
:   Title of the figure. Default '<code>var\_y\_name</code> vs. Time'.


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

#### Parameters

**```topic_name```** :&ensp;<code>str</code>
:   Name of the topic.

#### Returns

&ensp;<code>[CitrosDB](#citros\_data\_analysis.data\_access.CitrosDB "citros\_data\_analysis.data\_access.CitrosDB")</code>
:   CitrosDB with set 'topic' parameter.

#### Examples

To get data for topic name 'A':

```python
>>> df = citros.topic('A').data()
```

</details>


    
### Method `xy_plot` {#citros_data_analysis.data_access.CitrosDB.xy_plot}




```python
def xy_plot(
    self,
    ax,
    topic_name=None,
    var_x_name=None,
    var_y_name=None,
    sids=None,
    x_label=None,
    y_label=None,
    title_text=None
)
```


<details>
  <summary>Description</summary>

Plot <code>var\_y\_name</code> vs. <code>var\_x\_name</code> for each of the sids.

#### Parameters

**```ax```** :&ensp;<code>matplotlib.axes.Axes</code>
:   Figure axis to plot on.


**```topic_name```** :&ensp;<code>str</code>
:   Input topic name. If specified, will override value that was set by <code>topic()</code> method.


**```var_x_name```** :&ensp;<code>str</code>
:   Name of the variable to plot along x-axis.


**```var_y_name```** :&ensp;<code>str</code>
:   Name of the variable to plot along y-axis.


**```sids```** :&ensp;<code>list</code>, optional
:   List of the sids. If specified, will override values that were set by <code>sid()</code> method.
    If not specified, data for all sids is used.


**```x_label```** :&ensp;<code>str</code>, optional
:   Label to set to x-axis. Default <code>var\_x\_name</code>.


**```y_label```** :&ensp;<code>str</code>, optional
:   Label to set to y-axis. Default <code>var\_y\_name</code>.


**```title_text```** :&ensp;<code>str</code>, optional
:   Title of the figure. Default '<code>var\_y\_name</code> vs. <code>var\_x\_name</code>'.


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

#### Returns

&ensp;<code>str</code>
:   json_str


</details>
