---
# Display h3 headings
toc_max_heading_level: 3
sidebar_label: 'Documentation'
description: 'Documentation'
hide_title: true
---



    
## Module `citros_data_analysis.error_analysis` {#citros_data_analysis.error_analysis}







    
## Class `CitrosData` {#citros_data_analysis.error_analysis.CitrosData}





```python
class CitrosData(
    db=None,
    type_name='',
    units='',
    data_label='data',
    parameters=None,
    parameter_label='',
    sid_label='sid',
    omit_nan_rows=None,
    inf_vals=1e+308
)
```


<details>
  <summary>Description</summary>

Create CitrosData object, that allows to bin and interpolate data.

CitrosData object has two main attributes: 'data' - the vector of depending variables, 
and all other additional columns - 'addData'. Both 'data' and 'addData' attributes containes pandas.DataFrame.

---
#### Parameters

**```db```** :&ensp;**DataFrame** or **tuple** of **two DataFrames** or **None**, optional
:   If **db** is a DataFrame, column **data_label** is supposed to be a data sample and 
    set to a 'data' attribute of a CitrosData object.
    The additional information about data may be extracted from columns labeled as:
        'type_name' - specify type of the data, set to the 'type' attribute,
        'units' - data units, set to the 'units' attribute.
        'parameter_label' - column with dict, specifying the parameters.
            If present, the first row is set as parameters.
    All other columns are assigned to 'addData' attribute.


**```type_name```** :&ensp;**str**, optional
:   Specifies type of the data.


**```units```** :&ensp;**str**, optional
:   Specifies units of the data.


**```data_label```** :&ensp;**str** or **list** of **str**, default `'data'`
:   Specifies label of the data in DataFrame


**```parameters```** :&ensp;**dict**
:   Parameters. Mostly used in regression analysis.


**```parameter_label```** :&ensp;**str** or **list** of **str**
:   Specify label of a column in a pandas DataFrame, where the parameters are written as a dict.
    Used only if **db** is a pandas DataFrame and **parameters** is not specified.


**```sid_label```** :&ensp;**str**
:   label of the sim run id column (usually 'sim_run_id' or 'sid').


**```omit_nan_rows```** :&ensp;**bool**
:   If True, any rows containing one or more NaN values will be excluded from the analysis. 
    For multidimensional vectors, this means that the mean and covariance matrices will only be calculated 
    for rows that do not contain NaN values in all columns of the vector.
    Otherwise, if omit_nan_rows is set to False, columns are treated individually. 
    The mean values are computed over non-NaN values within each column, and the elements of the covariance 
    matrices are calculated pairwise, for rows without NaN values.
    For example, for 3-dimensional vector:


```code
+----+-----+-----+
| x  | y   | z   |
+====+=====+=====+
| 1  | 3   | NaN |
+----+-----+-----+
| 2  | NaN | 5   |
+----+-----+-----+
| 3  | 5   | 6   |
+----+-----+-----+
| 4  | 7   | 7   |
+----+-----+-----+
```

   if **omit_nan_rows** set True, the first and the second rows will be omitted from all calculations, while 
   in case **omit_nan_rows** set False, NaN values will be omitted only when the column is used in calculations. 
   For example, for mean calculations difference is the follows:

```code
    omit_nan_rows = True   omit_nan_rows = False
+-----+---+-----+      +-----+---+---+
|  x  | y | z   |      |  x  | y | z |
+=====+===+=====+      +=====+===+===+
| 3.5 | 6 | 6.5 |      | 2.5 | 5 | 6 |
+-----+---+-----+      +-----+---+---+
```     

**```inf_vals```** :&ensp;**None** or **float**, default **1e308**
:   If specified, all values from **data_label** column that exceed the provided value in absolute terms 
    will be treated as NaN values. If this functionality is not required, set inf_vals = None.





</details>






    
### Method `add_addData` {#citros_data_analysis.error_analysis.CitrosData.add_addData}




```python
def add_addData(
    self,
    column,
    column_label
)
```


<details>
  <summary>Description</summary>

Add column to 'addData' attribute.

---
#### Parameters

**```column```** :&ensp;`array-like object`
:   Column to add.


**```column_label```** :&ensp;**str**
:   Label of the new column in 'addData'.


</details>


    
### Method `bin_data` {#citros_data_analysis.error_analysis.CitrosData.bin_data}




```python
def bin_data(
    self,
    n_bins,
    param_label,
    min_lim=None,
    max_lim=None,
    show_fig=False
)
```


<details>
  <summary>Description</summary>

Bin values of column **param_label** in **n_bins** intervals, group data according to the binning and 
calculate mean data values of each group.

In order to establish a correspondence between the values of the data from different simulations, 
an independent variable **param_label** is selected and used to assign indexes. **param_label** values are divided into 
**n_bins** ranges, assigning index to each interval, and then for each simulation the averages of the data values 
is calculated in each bin.
'addData' and 'data' attributes of the new CitrosData object have two levels of indexes, 
with id values from binning as the first level and 'sid' as the second one.

---
#### Parameters

**```n_bins```** :&ensp;**int**, default **10**
:   Number of bins.


**```param_label```** :&ensp;**str**, default `'time'`
:   Label of column on the basis of which the indixes will be calculated.


**```min_lim```** :&ensp;**float**
:   The minimum value of the range for binning, **min_lim** < **max_lim**.
    If None then the minimum value of the entire range is selected.


**```max_lim```** :&ensp;**float**
:   The maximum value of the range for binning, **min_lim** < **max_lim**.
    If None then the maximum value of the entire range is selected.


**```show_fig```** :&ensp;**bool**, default **False**
:   If the histogram that represents the distibution of the values in **param_label** should be shown.

---
#### Returns

&ensp;**[CitrosData](#citros_data_analysis.error_analysis.CitrosData "citros_data_analysis.error_analysis.CitrosData")**
:   New CitrosData object with two levels of indexes in 'addData' and 'data' attributes.

---
#### Examples

Query some data from the topic 'A'

```python
>>> df = citros.topic('A').data(['data.x.x_1', 'data.time'])
>>> print(df)
```


Construct CitrosData object with one data-column 'data.x.x_1':

```python
>>> dataset = analysis.CitrosData(df, data_label=['data.x.x_1'], units = 'm')
```


Devide 'data.time' values in 50 bins and assign indexes to these intervals. For each simulation group 
'data.x.x_1' values according to the binning and calculate mean of the each group:

```python
>>> db = dataset.bin_data(n_bins = 50, param_label = 'data.time')
```


The result is a CitrosData object with two levels of indexes:

```python
>>> print(db.data)
                    data.x.x_1
data.time_id  sid            
0             1     0.00000
              2     -0.04460
              3     -0.07900
1             1     0.01600
...
```


```python
>>> print(db.addData)
                    data.time
data.time_id  sid           
0             1     8.458
              2     8.458
              3     8.458
1             1     24.774
...
```

</details>


    
### Method `drop_addData` {#citros_data_analysis.error_analysis.CitrosData.drop_addData}




```python
def drop_addData(
    self,
    column_label
)
```


<details>
  <summary>Description</summary>

Delete column from 'addData' attribute.

---
#### Parameters

**```column_label```** :&ensp;**str**
:   Label of the column to delete .


</details>


    
### Method `drop_parameter` {#citros_data_analysis.error_analysis.CitrosData.drop_parameter}




```python
def drop_parameter(
    self,
    key=None
)
```


<details>
  <summary>Description</summary>

Delete parameter labeled **key** and associated value.

---
#### Parameters

**```key```** :&ensp;**str**
:   Label of the parameter to remove.


</details>


    
### Method `get_statistics` {#citros_data_analysis.error_analysis.CitrosData.get_statistics}




```python
def get_statistics(
    self,
    return_format='pandas'
)
```


<details>
  <summary>Description</summary>

Return table with statistics for CitrosData object.

---
#### Parameters

**```return_format```** :&ensp;`{'pandas', 'citrosStat',}`, default `'pandas'`
:   Returning format.

---
#### Returns

**```Statistics```** :&ensp;**pandas.DataFrame** or **[CitrosStat](#citros_data_analysis.error_analysis.CitrosStat "citros_data_analysis.error_analysis.CitrosStat")**
:   Collected statistics.
    If **return_format** is 'pandas', then returns pandas.DataFrame with the following columns:


   - (1) the independent variable column, its label matches **x_label** attribute; 
   - (2) column with mean values;
   - (3) column with the covariance matrixes; 
   - (4) column with the square roots of the diagonal elements of the covariance matrix: ( sqrt(s1), sqrt(s2), sqrt(s3) ), 
     where s1,s2,s3 - diagonal of the covariance matrix. <br />
    If **return_format** is 'citrosStat', then returns CitrosStat object with 'x', 'mean', 'covar_matrix' and 'sigma' attributes,
    that corresponds to (1)-(4) items, but in the form of pandas.DataFrames.

---
#### See Also

**bin_data()**, **scale_data()**, **show_statistics()**

---
#### Examples

Import 'data_access' and 'error_analysis' modules and create CitrosDB object to query data:

```python
>>> from citros_data_analysis import data_access as da
>>> from citros_data_analysis import error_analysis as analysis
>>> citros = da.CitrosDB()
```


Let's consider a data of the topic 'A', json-data part of which has the following structure:

```python
data
{'x': {'x_1': -0.08, 'x_2': -0.002, 'x_3': 17.7}, 'time': 0.3}
{'x': {'x_1': 0.0, 'x_2': 0.08, 'x_3': 154.47}, 'time': 10.0}
...
...

Let's query data and pass it to CitrosData object to perform analysis.
It is possible to query all columns separetly:

```python
>>> df = citros.topic('A').data(['data.x.x_1', 'data.x.x_2', 'data.x.x_3', 'data.time'])
>>> print(df)
   sid   rid   time       topic   type   data.x.x_1   data.x.x_2   data.x.x_3   data.time
0  1     0     312751159  A       a      0.000        0.080        154.47       10.0
1  1     1     407264008  A       a      0.008        0.080        130.97       17.9
2  1     2     951279608  A       a      0.016        0.078        117.66       20.3
...
```


and define data labels for the CitrosData object as follows:

```python
>>> dataset = analysis.CitrosData(df,
...                               data_label = ['data.x.x_1', 'data.x.x_2', 'data.x.x_3'],
...                               units = 'm')
```


or query 'data.x' as a one column:

```python
>>> df = citros.topic('A').data(['data.x', 'data.time'])
>>> print(df)
   sid   rid   time       topic   type   data.x                                       data.time
0  1     0     312751159  A       a      {'x_1': 0.0, 'x_2': 0.08, 'x_3': 154.47}     10.0
1  1     1     407264008  A       a      {'x_1': 0.008, 'x_2': 0.08, 'x_3': 130.97}   17.9
2  1     2     951279608  A       a      {'x_1': 0.016, 'x_2': 0.078, 'x_3': 117.66}  20.3
...
```


and correspondingly set data_label:

```python
>>> dataset = analysis.CitrosData(df,
...                               data_label = 'data.x',
...                               units = 'm')
```


To analyze data of multiple simulations it is necessary to establish a correspondence between the values of the data 
from these different simulations. One approach is to select an independent variable, define a scale that is common 
to all simulations and assign indexes on this scale. Then, the values of variables from different simulations
will be connected by this independent variable.

There are two ways to perform index assignment: divide the independent variable into N ranges, 
assign an index to each interval, and calculate the averages of the data values for each simulation in each range, 
or scale the independent variable to the interval [0,1], define a new range of N points uniformly distributed from 0 to 1, 
and interpolate data points over this new interval. The first approach corresponds to the bin_data() method, while the second 
is implemented by the scale_data() method:

```python
>>> db = dataset.bin_data(n_bins = 50, param_label = 'data.time')
>>> #or
>>> db = dataset.scale_data(n_points = 50, param_label = 'data.time')
```


Let's assume that the last variant was chosen. And now get the statistics:

```python
>>> stat = db.get_statistics(return_format = 'citrosStat')
```


It returns CitrosStat object, that stores independent variable values, mean data values, covarian matrix and 
standard deviation (square root of the covariant matrix diogonal elements) for each index.

The mean data value, independent variable values and standard deviation are the pandas.DataFrames:

```python
>>> print(stat.mean)
               data.x.x_1   data.x.x_2  data.x.x_3
data.time_id                                    
0              -0.045667    0.044667    93.706667
1              -0.026038    0.059598    73.345027
...
```


```python
>>> print(stat.x)
               data.time
data.time_id           
0              0.000000
1              0.020408
...
```


```python
>>> print(stat.sigma)
                data.x.x_1  data.x.x_2  data.x.x_3
data.time_id                                    
0               0.041187    0.042158    69.647524
1               0.050354    0.026935    84.049381
2               0.049388    0.010733    40.279784
```


and the covariant matrix is a pandas.Series. Each its row contains N x N dimensional numpy.ndarray, where N
is a data dimension:

```python
>>> print(stat.covar_matrix.loc[0])
[[1.69633333e-03 1.54366667e-03 2.60583167e+00]
[1.54366667e-03 1.77733333e-03 2.93335333e+00]
[2.60583167e+00 2.93335333e+00 4.85077763e+03]]
```

</details>


    
### Method `scale_data` {#citros_data_analysis.error_analysis.CitrosData.scale_data}




```python
def scale_data(
    self,
    n_points,
    param_label,
    show_fig=False,
    intr_kind='linear'
)
```


<details>
  <summary>Description</summary>

Scale parameter **param_label** for each of the 'sid' and interpolate data on the new scale.

In order to establish a correspondence between the values of the data from different simulations, 
an independent variable **param_label** is selected and used to assign indexes. 
First the **param_label** interval is shifted and scaled in the way that the minimum value equals 0 and the maximum is 1.
Then the data is interpolated to a new scale, that consists of **n_points** evenly spaced points and spans from 0 to 1.
For each 'sid' this procedure is performed separetly.
'addData' and 'data' attributes of the new CitrosData object have two levels of indexes, 
with id values from scaling as the first level and 'sid' as the second one.

---
#### Parameters

**```n_points```** :&ensp;**int**
:   Number of points in a new scale, which will be used for interpolation.


**```param_label```** :&ensp;**str**
:   Label of the parameter to scale


**```show_fig```** :&ensp;**bool**, default **False**
:   If the figures with the results of interpolation should be shown.
    If the 'sid' exceed 5, only first 5 will be shown.
    If data consists of several vectors, for each of them the separate figure will be plotted.


**```intr_kind```** :&ensp;**str**, default `'linear'`
:   Type of the interpolation, see scipy.interpolate.interp1d.

---
#### Returns

&ensp;**[CitrosData](#citros_data_analysis.error_analysis.CitrosData "citros_data_analysis.error_analysis.CitrosData")**
:   CitrosData object with multiindexing: the first level stores ids of the points of the new scale, the second one - 'sid'.
    Values of the new scale are stored in 'addData' attribute.

---
#### Examples

Query some data from the topic 'A'

```python
>>> df = citros.topic('A').data(['data.x.x_1', 'data.time'])
>>> print(df)
```


Construct CitrosData object with one data-column 'data.x.x_1':

```python
>>> dataset = analysis.CitrosData(df, data_label=['data.x.x_1'], units = 'm')
```


Scale 'data.time' to [0, 1] interval, define a new range of 50 points uniformly distributed from 0 to 1, 
and interpolate data points over this new interval:

```python
>>> db = dataset.scale_data(n_points = 50, param_label = 'data.time')
```


The result is a CitrosData object with two levels of indexes:

```python
>>> print(db.data)
                    data.x.x_1
data.time_id  sid            
0             1      0.000000
              2     -0.057000
              3     -0.080000
1             1      0.025494
...
```


```python
>>> print(db.addData)
                    data.time
data.time_id  sid           
0             1     0.000000
              2     0.000000
              3     0.000000
1             1     0.020408
...
```

</details>


    
### Method `set_parameter` {#citros_data_analysis.error_analysis.CitrosData.set_parameter}




```python
def set_parameter(
    self,
    key=None,
    value=None,
    item=None
)
```


<details>
  <summary>Description</summary>

Set parameter value to a CitrosData object.

---
#### Parameters

**```key```** :&ensp;**str**
:   Label of the parameter.


**```value```** :&ensp;**int** or **float**
:   Parameter value.


**```item```** :&ensp;**dict**
:   Dictionary with parameters.


</details>


    
### Method `show_correlation` {#citros_data_analysis.error_analysis.CitrosData.show_correlation}




```python
def show_correlation(
    self,
    db2=None,
    x_col=0,
    y_col=0,
    slice_id=None,
    slice_val=None,
    n_std=3,
    bounding_error=False,
    fig=None,
    return_fig=False,
    display_id=True,
    **kwargs
)
```


<details>
  <summary>Description</summary>

Show data correlation for the given **slice_id**. 

Prepare data from one or more CitrosData objects and plot confidence ellipses for the specified id = **slice_id**.
If the data stored in CitrosData object **db** is multidimensional, then **x_colNumber** and **y_colNumber** must be provided.
If the data from another CitrosData objects is used, the latter must be provided in **db2**. Then the data from **db** 
is supposed to be plotted along x-axis and the data from **db2** is supposed to be plotted along y-axis.

---
#### Parameters

**```db2```** :&ensp;**[CitrosData](#citros_data_analysis.error_analysis.CitrosData "citros_data_analysis.error_analysis.CitrosData")**
:   Additional CitrosData object.


**```x_col```** :&ensp;`int >=0` or **str**, optional
:   If int - index of column to plot along x axis, >=0.
    If str - label of the column to plot along y axis
    If data is multidimensional, must be specified, otherwise data is supposed to be 1-dimensional.


**```y_col```** :&ensp;`int >=0`  or **str**, optional
:   If int - index of column to plot along y axis, >=0.
    If str - label of the column to plot along y axis
    If data is multidimensional, must be specified, otherwise data is supposed to be 1-dimensional.


**```slice_id```** :&ensp;**int**
:   id of the slice.


**```slice_val```** :&ensp;**float**
:   Value, for which the nearest slice_id is search.
    Used only if slice_id is None.


**```n_std```** :&ensp;**list** or **int**, default **3**
:   Radius or list of radii of the confidence ellipses in sigmas, 3 by default.


**```bounding_error```** :&ensp;**bool**, default **False**
:   If the bounding error should be depicted.


**```fig```** :&ensp;**matplotlib.figure.Figure**, optional
:   figure to plot on. If None, then the new one is created.


return_fig : bool, default False.
    If the fig, ax should be returned.
**```display_id```** :&ensp;**bool**, default **True**
:   Whether to print the pair of **slice_id** **slice_val** or not.

---
#### Other Parameters

**```**kwargs```**
:   see matplotlib.patches.Ellipse.

---
#### Returns

**```fig```** :&ensp;**matplotlib.figure.Figure**
:   if **return_fig** set to True


**```ax```** :&ensp;**matplotlib.axes.Axes**
:   if **return_fig** set to True

---
#### Examples

Import 'data_access' and 'error_analysis' modules and create CitrosDB object to query data:

```python
>>> from citros_data_analysis import data_access as da
>>> from citros_data_analysis import error_analysis as analysis
>>> citros = da.CitrosDB()
```


For topic 'B' query json-data column 'data.x.x_1', 'data.x.x_2' and 'data.time':

```python
>>> df = citros.topic('B').data(['data.x.x_1', 'data.x.x_2', 'data.time'])
```


Construct CitrosData object with 2 data-columns 'data.x.x_1', 'data.x.x_2':

```python
>>> dataset = analysis.CitrosData(df, data_label=['data.x.x_1', 'data.x.x_2'], units = 'm')
```


Use method scale_data() or bin_data() to get correspondence between different simulation
and assign indexes to 'data.time' axis:

```python
>>> db_sc = dataset.scale_data(n_points = 20, 
...                            param_label = 'data.time', 
...                            show_fig = False)
```


Plot correlation plot for the index = 5:

```python
>>> db_sc.show_correlation(x_col = 'data.x.x_2',
...                        y_col = 'data.x.x_1',
...                        slice_id = 5,
...                        n_std = [1,2,3],
...                        bounding_error= False)
slice_id = 5,
slice_val = 0.2632
```

</details>


    
### Method `show_statistics` {#citros_data_analysis.error_analysis.CitrosData.show_statistics}




```python
def show_statistics(
    self,
    fig=None,
    show_fig=True,
    return_fig=False,
    n_std=3,
    fig_title='Statistics',
    std_color='r',
    connect_nan_std=True,
    std_area=False,
    std_lines=True
)
```


<details>
  <summary>Description</summary>

Collect statistics for CitrosData object and plot it.

---
#### Parameters

**```fig```** :&ensp;**matplotlib.figure.Figure**
:   figure to plot on. If None, the new one will be created.


**```show_fig```** :&ensp;**bool**
:   If the fugure should be shown, True by default.


**```return_fig```** :&ensp;**bool**
:   If the figure parameters fig, ax should be returned; 
    fig is matplotlib.figure.Figure and ax is matplotlib.axes.Axes


**```n_std```** :&ensp;**int**, default **3**
:   Error interval to display in standard deviations.


**```fig_title```** :&ensp;**str**, default `'Statistics'`
:   Title of the figure.


**```std_color```** :&ensp;**str**, default `'r'`
:   Color for dispalying standard deviations, red by default.


**```connect_nan_std```** :&ensp;**bool**, default **True**
:   If True, all non-NaN values in standard deviation boundary line are connected, resulting in a continuous line. 
    Otherwise, breaks are introduced in the standard deviation line whenever NaN values are encountered.


**```std_area```** :&ensp;**bool**, default **False**
:   Fill area within **n_std**-standard deviation lines with color.


**```std_lines```** :&ensp;**bool**, default **True**
:   If False, remove standard deviation boundary lines.

---
#### Returns

**```fig```** :&ensp;**matplotlib.figure.Figure**
:   if **return_fig** set to True


**```ax```** :&ensp;**numpy.ndarray** of **matplotlib.axes.Axes**
:   if **return_fig** set to True

---
#### See Also

**get_statistics()**, **bin_data()**, **scale_data()**

---
#### Examples

Import 'data_access' and 'error_analysis' modules and create CitrosDB object to query data:

```python
>>> from citros_data_analysis import data_access as da
>>> from citros_data_analysis import error_analysis as analysis
>>> citros = da.CitrosDB()
```


Download json-data column 'data.x', that containes data.x.x_1, data.x.x_2 and data.x.x_3 and column 'data.time':

```python
>>> df = citros.topic('A').data(['data.x', 'data.time'])
```


Construct CitrosData object with 3 data-columns from 'data.x':

```python
>>> dataset = analysis.CitrosData(df, data_label=['data.x'], units = 'm')
```


Use method scale_data() or bin_data() to get correspondence between different simulation:

```python
>>> db_sc = dataset.scale_data(n_points = 150, 
                               param_label = 'data.time', 
                               show_fig = False)
```


Show statistics plot:

```python
>>> db_sc.show_statistics()
```

</details>


    
### Method `to_pandas` {#citros_data_analysis.error_analysis.CitrosData.to_pandas}




```python
def to_pandas(
    self
)
```


<details>
  <summary>Description</summary>

Concatenate **data** and **addData** attributes and return the result table as a pandas.DataFrame.

---
#### Returns

&ensp;**pandas.DataFrame**
:   Concatenated table.


</details>


    
## Class `CitrosDataArray` {#citros_data_analysis.error_analysis.CitrosDataArray}





```python
class CitrosDataArray(
    dbs=None
)
```


<details>
  <summary>Description</summary>

Store CitrosData objects in a "dbs" attribute for regression analysis.

---
#### Parameters

**```dbs```** :&ensp;**list**
:   list of CitrosData objects





</details>






    
### Method `add_db` {#citros_data_analysis.error_analysis.CitrosDataArray.add_db}




```python
def add_db(
    self,
    db
)
```


<details>
  <summary>Description</summary>

Add one CitrosData object to CitrosDataArray.

---
#### Parameters

**```db```** :&ensp;**[CitrosData](#citros_data_analysis.error_analysis.CitrosData "citros_data_analysis.error_analysis.CitrosData")**
:   CitrosData object to add to storage.


</details>


    
### Method `add_dbs` {#citros_data_analysis.error_analysis.CitrosDataArray.add_dbs}




```python
def add_dbs(
    self,
    dbs
)
```


<details>
  <summary>Description</summary>

Add list of CitrosData objects to CitrosDataArray.

---
#### Parameters

**```dbs```** :&ensp;**list **
:   list of CitrosData objects to add to storage.


</details>


    
### Method `drop_db` {#citros_data_analysis.error_analysis.CitrosDataArray.drop_db}




```python
def drop_db(
    self,
    value
)
```


<details>
  <summary>Description</summary>

Remove CitrosData object from CitrosDataArray.

If **value** is an int, then preformes removing by index, 
if **value** is a CitrosData object, then removes it if it exists in CitrosDataArray.

---
#### Parameters

**```value```** :&ensp;**int** or **[CitrosData](#citros_data_analysis.error_analysis.CitrosData "citros_data_analysis.error_analysis.CitrosData")**
:   &nbsp;


</details>


    
### Method `get_prediction` {#citros_data_analysis.error_analysis.CitrosDataArray.get_prediction}




```python
def get_prediction(
    self,
    parameters,
    method='poly',
    n_poly=2,
    activation='relu',
    max_iter=500,
    solver='lbfgs',
    hidden_layer_sizes=(10,),
    alpha=1e-16,
    fig=None,
    show_fig=False,
    return_fig=False,
    **kwargs
)
```


<details>
  <summary>Description</summary>

Show the predictions based on the results of the regression solution, neural net or gaussian mixture model.

---
#### Parameters

**```parameters```** :&ensp;**dict**
:   Names of the independent parameters and their values to calculate the prediction.


**```method```** :&ensp;**str** or **list** of **str**, default `'regression'`
:   If the **method** is 'poly', the polinomial regression is solved.
    If the **method** is 'neural_net', the solution is finding by sklearn.neural_network.MLPRegressor.
    If the **method** is 'gmm', the gaussian mixture model is built and used for the prediction.


**```n_poly```** :&ensp;**int**, default **2**
:   Only used if **method** = 'poly'.
    The highest degree of the polynomial (1 for linear, 2 for quadratic, etc).


**```activation```** :&ensp;`{'relu', 'identity', 'logistic'` or `'tanh'}`, default `'relu'`
:   Only used if **method** = 'neural_net'.
    Activation function for the hidden layer, see sklearn.neural_network.MLPRegressor


**```max_iter```** :&ensp;**int**, default **500**
:   Only used if **method** = 'neural_net'.
    Maximum number of iterations.


**```solver```** :&ensp;`{'lbfgs', 'sgd', 'adam'}`, default `'lbfgs'`
:   Only used if **method** = 'neural_net'.
    The solver for weight optimization.


**```hidden_layer_sizes```** :&ensp;`array-like` of `shape(n_layers - 2,)`, default=**(10,)**
:   Only used if **method** = 'neural_net'.
    The ith element represents the number of neurons in the ith hidden layer.


**```alpha```** :&ensp;**float**, default `1e-16`
:   Only used if **method** = 'gmm'.
    Value of the covariance element of parameters.


**```fig```** :&ensp;**matplotlib.figure.Figure**, optional
:   figure to plot on. If None, then the new one is created.


**```show_fig```** :&ensp;**bool**, default **True**
:   If the figure will be shown.


**```return_fig```** :&ensp;**bool**, default **False**
:   If True, the figure and ax (or list of ax) will be returned.


**```**kwargs```**
:   other keyword arguments for **method** = 'neural_net', see sklearn.neural_network.MLPRegressor.

---
#### Returns

**```result```** :&ensp;**pandas.DataFrame**
:   Predicted table


**```fig```** :&ensp;**matplotlib.figure.Figure**
:   if **return_fig** set to True


**```ax```** :&ensp;**matplotlib.axes.Axes** or **list** of **matplotlib.axes.Axes**
:   if **return_fig** set to True

---
#### Examples

Create CitrosDataArray object:

```python
>>> db_array = analysis.CitrosDataArray()
```


Let's assume that for the topic 'A' there is simulations for the four different values of the some parameter 't', 
that is written in json-data column 'data.t'. To get list of the 'data.t' parameters get_unique_values() 
method may be used:

```python
>>> list_t = citros.topic('A').get_unique_values('data.t')
>>> print(list_t)
[-1.5, 0, 2.5, 4]
```


Let's find prediction for the values of the 'data.x.x_1' json-column for the case when 'data.t' equals 1.
To query data for each of these parameter values, set it as parameter, assign indexes over 'data.time' axis to set
correspondence between different simulations and pass the result to CitrosDataArray that we created:

```python
>>> for t in list_t:
...     #query data
...     df = citros.topic('A')\
...                .set_filter({'data.t': [t]})\
...                .data(['data.x.x_1', 'data.time', 'data.t'])
...
...     #create CitrosData object and set 'data.t' as a parameter.
...     dataset = analysis.CitrosData(df,  
...                                  data_label=['data.x.x_1'],
...                                  units = 'm', 
...                                  parameter_label = ['data.t'])
...
...     #scale over 'data.time'
...     db_sc = dataset.scale_data(n_points = 100, 
...                                param_label = 'data.time', 
...                                show_fig = False)
...
...     #store in CitrosDataArray by add_db() method
...     db_array.add_db(db_sc)
```


Get the prediction with 'poly' method:

```python
>>> result = db_array.get_prediction(parameters = {'data.t': 1},
...                                  method = 'poly', 
...                                  n_poly = 2,
...                                  show_fig = True)
>>> print(result)
    data.time   data.x.x_1
0       0.000000        1.155301
1       0.010101        1.145971
2       0.020202        1.232255
...
```

</details>


    
## Class `CitrosStat` {#citros_data_analysis.error_analysis.CitrosStat}





```python
class CitrosStat(
    F,
    labels,
    x_label
)
```


<details>
  <summary>Description</summary>

Object to store statistics.

---
#### Parameters

**```F```** :&ensp;**pandas.DataFrame**
:   Table with statistics.


**```labels```** :&ensp;`array-like`
:   Labels of the data columns.


**```x_label```** :&ensp;**str**
:   Label of the independent variable.

---
#### Attributes

**```x```** :&ensp;**pandas.DataFrame**
:   Table with independent variable.


**```mean```** :&ensp;**pandas.DataFrame**
:   Table with mean values. If statistics was collected for a vector, columns correspond to vector elements.


**```covar_matrix```** :&ensp;**pandas.DataFrame**
:   Table with the covariance matrixes. If statistics was collected for a vector, columns correspond to vector elements.


**```sigma```** :&ensp;**pandas.DataFrame**
:   Table with the square roots of the diagonal elements of the covariance matrix. 
    If statistics was collected for a vector, columns correspond to vector elements.





</details>






    
### Method `plot` {#citros_data_analysis.error_analysis.CitrosStat.plot}




```python
def plot(
    self,
    fig=None,
    show_fig=True,
    return_fig=False,
    n_std=3,
    fig_title='Statistics',
    std_color='r'
)
```


<details>
  <summary>Description</summary>

Plot mean values and standard deviations.

---
#### Parameters

**```fig```** :&ensp;**matplotlib.figure.Figure**
:   figure to plot on. If None, the new one will be created.


**```show_fig```** :&ensp;**bool**
:   If the fugure should be shown, True by default.


**```return_fig```** :&ensp;**bool**
:   If the figure parameters fig, ax should be returned; 
    fig is matplotlib.figure.Figure and ax is matplotlib.axes.Axes


**```n_std```** :&ensp;**int**, default **3**
:   Error interval to display, specified in standard deviations.


**```fig_title```** :&ensp;**str**, default `'Statistics'`
:   Title of the figure.


**```std_color```** :&ensp;**str**, default `'r'`
:   Color for dispalying standard deviations, red by default.

---
#### Returns

**```fig```** :&ensp;**matplotlib.figure.Figure**
:   if **return_fig** set to True


**```ax```** :&ensp;**list** of **matplotlib.axes.Axes**
:   if **return_fig** set to True


</details>


    
### Method `to_pandas` {#citros_data_analysis.error_analysis.CitrosStat.to_pandas}




```python
def to_pandas(
    self
)
```


<details>
  <summary>Description</summary>

Convert CitrosStat object back to pandas DataFrame.

---
#### Returns

&ensp;**pandas.DataFrame**
:   Converted to pandas DataFrame.


</details>