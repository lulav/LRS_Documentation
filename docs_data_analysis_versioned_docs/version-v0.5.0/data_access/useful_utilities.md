---
toc_max_heading_level: 4
hide_title: true
sidebar_position: 8
sidebar_label: 'Useful utilities'
---

## Useful utilities

Some useful methods to get more information about the data.

### Unique values

Method [**get_unique_values(column_names)**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.get_unique_values) of the [**CitrosDB**](getting_started.md#connection-to-the-database) object is used to get the unique values or combination of values of the columns `column_names`.

:::note
Like in the case of [data querying](query_data.md#query-data), such methods as [**topic()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.topic), [**rid()**](query_data.md#rid-constraints), [**sid()**](query_data.md#sid-constraints), [**time()**](query_data.md#time-constraints) and [**set_filter()**](query_data.md#json-data-constraints) may be applied too to define constraints.
:::

- Get the unique combination of "topic" and "type" and show it by **prettytable**:

```python
>>> result = citros.get_unique_values(column_names = ['topic', 'type_name'])
>>> table = PrettyTable(field_names=column_names, align='r')
>>> table.add_rows(result)
>>> print(table)
```

```text
+-------+------+
| topic | type |
+-------+------+
|     B |    b |
|     A |    a |
|     D |    d |
|     C |    c |
+-------+------+
```

- Get the unique values of the column "type", but only for specific "time" and "topic" values, for example 100 < "time" <= 200, "topic" = 'A':

```python
#get the unique values
>>> result = citros.topic('A').set_filter({'time': {'>': 100, '<=': 200}})\
                              .get_unique_values(column_names = ['type'])

#print
>>> table = PrettyTable(field_names=column_names, align='r')
>>> table.add_rows(result)
>>> print(table)
```

```text
+------+
| type |
+------+
|    a |
+------+
```

Another way to apply constraints is to use argument `filter_by`, that has the same syntax as [**set_filter()**](query_data.md#json-data-constraints) method. This way, the query from the previous example will look like:

```python
>>> result = citros.get_unique_values(column_names = ['type'], 
                                  filter_by = {'topic': 'A', 'time': {'>': 100, '<=': 200}})
```
:::note
Constraints passed by `filter_by` will override those defined by [**topic()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.topic), [**rid()**](query_data.md#rid-constraints), [**sid()**](query_data.md#sid-constraints), [**time()**](query_data.md#time-constraints) and [**set_filter()**](query_data.md#json-data-constraints) methods.
:::

### Maximum and minimum values

To find the maximum and the minimum values of the `column_name` methods [**get_max_value(column_name)**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.get_max_value) and [**get_min_value(column_name)**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.get_min_value) are used.

:::note
Use [**topic()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.topic), [**rid()**](query_data.md#rid-constraints), [**sid()**](query_data.md#sid-constraints), [**time()**](query_data.md#time-constraints) and [**set_filter()**](query_data.md#json-data-constraints) methods to apply constraints.
:::

Find the maximum and the minimum values of the column 'rid' for topic 'A':

```python
#specify column
>>> column_name = 'rid'

#get max and min value for topic 'A":
>>> result_max = citros.topic('A').get_max_value(column_name)
>>> result_min = citros.topic('A').get_min_value(column_name)

#print
>>> print(f"max value of the column '{column_name}' : {result_max}")
>>> print(f"min value of the column '{column_name}' : {result_min}")
```
The output is:
```text
max value of the column 'rid' : 163
min value of the column 'rid' : 0
```

Another way to apply constraints is to use argument `filter_by`, that has the same syntax as [**set_filter()**](query_data.md#json-data-constraints) method. If we rewrite the query from the previous example with `filter_by` argument:

```python
>>> result_max = citros.get_max_value(column_name, filter_by = {'topic': 'A'})
>>> result_min = citros.get_min_value(column_name, filter_by = {'topic': 'A'})
```

:::note
Constraints passed by `filter_by` will override those defined by [**topic()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.topic), [**rid()**](query_data.md#rid-constraints), [**sid()**](query_data.md#sid-constraints), [**time()**](query_data.md#time-constraints) and [**set_filter()**](query_data.md#json-data-constraints) methods.
:::

### Number of messages

To calculate the number of messages in the column `column_name` method [**get_counts(column_name, group_by = None)**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.get_counts) is used.

:::note
Methods [**topic()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.topic), [**rid()**](query_data.md#rid-constraints), [**sid()**](query_data.md#sid-constraints), [**time()**](query_data.md#time-constraints) and [**set_filter()**](query_data.md#json-data-constraints) may be used to apply constraints.
:::

To display the total number of rows in the table call [**get_counts(column_name, group_by = None)**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.get_counts) without arguments: 

```python
>>> citros.get_counts()
```
```text
[(2000,)]
```
It is possible to group the results, for example by topics:
```python
>>> citros.get_counts(group_by = 'topic')
```
```text
[('D', 519), ('A', 474), ('B', 494), ('C', 513)]
```
or get the number of rows for the specific topic:
```python
>>> citros.topic('A').get_counts()
```
```text
[(474,)]
```

Let's find the number of the rows in column "rid", for messages which meet the following requirements: "type" is 'a' or 'b' and "time" <= 150. If we would like to see counts for each "type" separately, `group_by` argument may be used:

```python
#specify name of the column
>>> column_name = 'sid'

#Set "time" <= 150 and set "type" to be 'a' or 'b', group the counts by 'type':
>>> counts = citros.time(end = 150).set_filter({'type': ['a', 'b']})\
                                   .get_counts(column_name, group_by = ['type'])

#print the result:
>>> print(f"number of messages in column '{column_name}':")
>>> table = PrettyTable(field_names=['type', 'counts'], align='r')
>>> table.add_rows(counts)
>>> print(table)
```

```text
number of messages in column 'sid':
+------+--------+
| type | counts |
+------+--------+
|    b |    494 |
|    a |    474 |
+------+--------+
```

Another way to apply constraints is to use argument `filter_by`, that has the same syntax as [**set_filter()**](query_data.md#json-data-constraints) method. This way, the query from the previous example will look like:

```python
>>> counts = citros.get_counts(column_name, 
                      group_by = ['type'],
                      filter_by = {'time': {'<=': 150}, 'type': ['a', 'b']})
```

:::note
Constraints passed by `filter_by` will override those defined by [**topic()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.topic), [**rid()**](query_data.md#rid-constraints), [**sid()**](query_data.md#sid-constraints), [**time()**](query_data.md#time-constraints) and [**set_filter()**](query_data.md#json-data-constraints) methods.
:::

### Number of the unique values

To see the number of the unique values, the method [**get_unique_counts(column_name, group_by = None)**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.get_unique_counts) is used.

:::note
[**topic()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.topic), [**rid()**](query_data.md#rid-constraints), [**sid()**](query_data.md#sid-constraints), [**time()**](query_data.md#time-constraints) and [**set_filter()**](query_data.md#json-data-constraints) methods are used to define constraints.
:::

Let's find the number of unique values in column "rid", for messages, which meet the following requirements: "type" is 'a' or 'b' and "time" <= 150. To see counts for each "type" separately, `group_by` argument may be used:

```python
#sfecify name of the column
column_name = 'sid'

#Set "time" <= 150 and set "type" to be 'a' or 'b', group the counts by 'type':
>>> counts = citros.time(end = 150)\
                   .set_filter({'type': ['a', 'b']})\
                   .get_unique_counts(column_name, group_by = ['type'])

#print the result:
>>> print(f"number of unique values in column '{column_name}':")
>>> table = PrettyTable(field_names=['type', 'unique_counts'], align='r')
>>> table.add_rows(counts)
>>> print(table)
```

```text
number of unique values in column 'sid':
+------+---------------+
| type | unique_counts |
+------+---------------+
|    a |             3 |
|    b |             3 |
+------+---------------+
```

Another way to apply constraints is to use argument `filter_by`, that has the same syntax as [**set_filter()**](query_data.md#json-data-constraints) method. This way, the query from the previous example will look like:

```python
>>> counts = citros.get_unique_counts(column_name, group_by = ['type'], 
                            filter_by = {'time': {'<=': 150}, 'type': ['a', 'b']})
```

:::note
Constraints passed by `filter_by` will override those defined by [**topic()**](../documentation/data_access/citros_db.md#citros_data_analysis.data_access.citros_db.CitrosDB.topic), [**rid()**](query_data.md#rid-constraints), [**sid()**](query_data.md#sid-constraints), [**time()**](query_data.md#time-constraints) and [**set_filter()**](query_data.md#json-data-constraints) methods.
:::