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
    data_label='sample',
    parameters=None,
    parameter_label='',
    sid_label='sid'
)
```


<details>
  <summary>Description</summary>

Create CitrosData object, that allows to bin and interpolate data.

CitrosData object has two main attributes: 'data' - the vector of depending variables, 
and all other additional columns - 'addData'. Both 'data' and 'addData' attributes containes pandas.DataFrame.

#### Parameters

**```db```** :&ensp;<code>DataFrame</code> or <code>tuple</code> of <code>two DataFrames</code> or <code>None</code>, optional
:   If <code>db</code> is a DataFrame, column <code>data\_label</code> is supposed to be a data sample and 
    set to a 'data' attribute of a CitrosData object.
    The additional information about data may be extracted from columns labeled as:
        'type_name' - specify type of the data, set to the 'type' attribute,
        'units' - data units, set to the 'units' attribute.
        'parameter_label' - column with dict, specifying the parameters.
            If present, the first row is set as parameters.
    All other columns are assigned to 'addData' attribute.


**```type_name```** :&ensp;<code>str</code>, optional
:   Specifies type of the data.


**```units```** :&ensp;<code>str</code>, optional
:   Specifies units of the data.


**```data_label```** :&ensp;<code>str</code> or <code>list</code> of <code>str</code>, default `'sample'`
:   Specifies label of the data in DataFrame


**```parameters```** :&ensp;<code>dict</code>
:   Parameters. Mostly used in regression analysis.


**```parameter_label```** :&ensp;<code>str</code> or <code>list</code> of <code>str</code>
:   Specify label of a column in a pandas DataFrame, where the parameters are written as a dict.
    Used only if <code>db</code> is a pandas DataFrame and <code>parameters</code> is not specified.


**```sid_label```** :&ensp;<code>str</code>
:   label of the sim run id column (usually 'sim_run_id' or 'sid').





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

#### Parameters

**```column```** :&ensp;`array-like object`
:   Column to add.


**```column_label```** :&ensp;<code>str</code>
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

Bin values of column <code>param\_label</code> in <code>n\_bins</code> intervals, group data according to the binning and 
calculate mean values of each group.

'addData' and 'data' attributes of the new CitrosData object have two levels of indexes, 
with id values from binning as the first level and 'sid' as the second one.

#### Parameters

**```n_bins```** :&ensp;<code>int</code>, default <code>10</code>
:   Number of bins.


**```param_label```** :&ensp;<code>str</code>, default `'time'`
:   Label of column on the basis of which the indixes will be calculated.
    This label is supposed to be 'time', 'h' or 'd'.


**```min_lim```** :&ensp;<code>float</code>
:   The minimum value of the range for binning, <code>min\_lim</code> < <code>max\_lim</code>.
    If None then the minimum value of the entire range is selected.


**```max_lim```** :&ensp;<code>float</code>
:   The maximum value of the range for binning, <code>min\_lim</code> < <code>max\_lim</code>.
    If None then the maximum value of the entire range is selected.


**```show_fig```** :&ensp;<code>bool</code>, default <code>False</code>
:   If the histogram that represents the distibution of the values in <code>param\_label</code> should be shown.

#### Returns

&ensp;<code>[CitrosData](#citros\_data\_analysis.error\_analysis.CitrosData "citros\_data\_analysis.error\_analysis.CitrosData")</code>
:   New CitrosData object with two levels of indexes in 'addData' and 'data' attributes.


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

#### Parameters

**```column_label```** :&ensp;<code>str</code>
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

Delete parameter labeled <code>key</code> and associated value.

#### Parameters

**```key```** :&ensp;<code>str</code>
:   Label of the parameter to remove.


</details>


    
### Method `get_statistics` {#citros_data_analysis.error_analysis.CitrosData.get_statistics}




```python
def get_statistics(
    self
)
```


<details>
  <summary>Description</summary>

Return table with statistics for CitrosData object.

#### Returns

**```Statistics```** :&ensp;<code>pandas.DataFrame</code>
:   table with collected statistics. Contains the following columns:
    (1) the independent variable column, its label matches <code>x\_label</code> attribute; (2) column with mean values;
    (3) column with the covariance matrixes; (4) column with the square roots of the diagonal elements 
    of the covariance matrix: ( sqrt(s1), sqrt(s2), sqrt(s3) ),  where s1,s2,s3 - diagonal of the covariance matrix.


</details>


    
### Method `scale_data` {#citros_data_analysis.error_analysis.CitrosData.scale_data}




```python
def scale_data(
    self,
    n_points,
    param_label,
    show_fig=True,
    intr_kind='linear'
)
```


<details>
  <summary>Description</summary>

Scale parameter <code>param\_label</code> for each of the 'sid' and interpolate data on the new scale.

First the <code>param\_label</code> interval is shifted and scaled in the way that the minimum value equals 0 and the maximum is 1.
Then the data is interpolated to a new scale, consisted of <code>n\_points</code> evenly spaced points.
The new scale spans from 0 to 1.
For each 'sid' this procedure is performed separetly.

#### Parameters

**```n_points```** :&ensp;<code>int</code>
:   Number of points in a new scale, which will be used for interpolation.


**```param_label```** :&ensp;<code>str</code>
:   Label of the parameter to scale


**```show_fig```** :&ensp;<code>bool</code>
:   If the figures with the results of interpolation should be shown.
    If the 'sid' exceed 5, only first 5 will be shown.
    If data consists of several vectors, for each of them the separate figure will be plotted.


**```intr_kind```** :&ensp;<code>str</code>, default `'linear'`
:   Type of the interpolation, see scipy.interpolate.interp1d.

#### Returns

&ensp;<code>[CitrosData](#citros\_data\_analysis.error\_analysis.CitrosData "citros\_data\_analysis.error\_analysis.CitrosData")</code>
:   CitrosData object with multiindexing: the first level stores ids of the points of the new scale, the second one - 'sid'.
    Values of the new scale are stored in 'addData' attribute.


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

#### Parameters

**```key```** :&ensp;<code>str</code>
:   Label of the parameter.


**```value```** :&ensp;<code>int</code> or <code>float</code>
:   Parameter value.


**```item```** :&ensp;<code>dict</code>
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

Show data correlation for the given <code>slice\_id</code>. 

Prepare data from one or more CitrosData objects and plot confidence ellipses for the specified id = <code>slice\_id</code>.
If the data stored in CitrosData object <code>db</code> is multidimensional, then <code>x\_colNumber</code> and <code>y\_colNumber</code> must be provided.
If the data from another CitrosData objects is used, the latter must be provided in <code>db2</code>. Then the data from <code>db</code> 
is supposed to be plotted along x-axis and the data from <code>db2</code> is supposed to be plotted along y-axis.

#### Parameters

**```db2```** :&ensp;<code>[CitrosData](#citros\_data\_analysis.error\_analysis.CitrosData "citros\_data\_analysis.error\_analysis.CitrosData")</code>
:   Additional CitrosData object.


**```x_col```** :&ensp;`int >=0` or <code>str</code>, optional
:   If int - index of column to plot along x axis, >=0.
    If str - label of the column to plot along y axis
    If data is multidimensional, must be specified, otherwise data is supposed to be 1-dimensional.


**```y_col```** :&ensp;`int >=0`  or <code>str</code>, optional
:   If int - index of column to plot along y axis, >=0.
    If str - label of the column to plot along y axis
    If data is multidimensional, must be specified, otherwise data is supposed to be 1-dimensional.


**```slice_id```** :&ensp;<code>int</code>
:   id of the slice.


**```slice_val```** :&ensp;<code>float</code>
:   Value, for which the nearest slice_id is search.
    Used only if slice_id is None.


**```n_std```** :&ensp;<code>list</code> or <code>int</code>, default <code>3</code>
:   Radius or list of radii of the confidence ellipses in sigmas, 3 by default.


**```bounding_error```** :&ensp;<code>bool</code>, default <code>False</code>
:   If the bounding error should be depicted.


**```fig```** :&ensp;<code>matplotlib.figure.Figure</code>, optional
:   figure to plot on. If None, then the new one is created.


return_fig : bool, default False.
    If the fig, ax should be returned.
**```display_id```** :&ensp;<code>bool</code>, default <code>True</code>
:   Whether to print the pair of <code>slice\_id</code> <code>slice\_val</code> or not.


**```**kwargs```**
:   see matplotlib.patches.Ellipse.

#### Returns

**```fig```** :&ensp;<code>matplotlib.figure.Figure</code>
:   if <code>return\_fig</code> set to True


**```ax```** :&ensp;<code>matplotlib.axes.Axes</code>
:   if <code>return\_fig</code> set to True


</details>


    
### Method `show_statistics` {#citros_data_analysis.error_analysis.CitrosData.show_statistics}




```python
def show_statistics(
    self,
    fig=None,
    show_fig=True,
    return_fig=False
)
```


<details>
  <summary>Description</summary>

Collect all statistics for CitrosData object and plot it.

#### Parameters

**```fig```** :&ensp;<code>matplotlib.figure.Figure</code>
:   figure to plot on. If None, the new one will be created.


**```show_fig```** :&ensp;<code>bool</code>
:   If the fugure should be shown, True by default.


**```return_fig```** :&ensp;<code>bool</code>
:   If the figure parameters fig, ax should be returned; 
    fig is matplotlib.figure.Figure and ax is matplotlib.axes.Axes

#### Returns

**```fig```** :&ensp;<code>matplotlib.figure.Figure</code>
:   if <code>return\_fig</code> set to True


**```ax```** :&ensp;<code>list</code> of <code>matplotlib.axes.Axes</code>
:   if <code>return\_fig</code> set to True


</details>


    
### Method `to_pandas` {#citros_data_analysis.error_analysis.CitrosData.to_pandas}




```python
def to_pandas(
    self
)
```


<details>
  <summary>Description</summary>

Concatenate <code>data</code> and <code>addData</code> attributes and return the result table as a pandas.DataFrame.

#### Returns

&ensp;<code>pandas.DataFrame</code>
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

#### Parameters

**```dbs```** :&ensp;<code>list</code>
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

#### Parameters

**```db```** :&ensp;<code>[CitrosData](#citros\_data\_analysis.error\_analysis.CitrosData "citros\_data\_analysis.error\_analysis.CitrosData")</code>
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

#### Parameters

**```dbs```** :&ensp;<code>list </code>
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

If <code>value</code> is an int, then preformes removing by index, 
if <code>value</code> is a CitrosData object, then removes it if it exists in CitrosDataArray.

#### Parameters

**```value```** :&ensp;<code>int</code> or <code>[CitrosData](#citros\_data\_analysis.error\_analysis.CitrosData "citros\_data\_analysis.error\_analysis.CitrosData")</code>
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

#### Parameters

**```parameters```** :&ensp;<code>dict</code>
:   Names of the independent parameters and their values to calculate the prediction.


**```method```** :&ensp;<code>str</code> or <code>list</code> of <code>str</code>, default `'regression'`
:   If the <code>method</code> is 'poly', the polinomial regression is solved.
    If the <code>method</code> is 'neural_net', the solution is finding by sklearn.neural_network.MLPRegressor.
    If the <code>method</code> is 'gmm', the gaussian mixture model is built and used for the prediction.


**```n_poly```** :&ensp;<code>int</code>, default <code>2</code>
:   Only used if <code>method</code> = 'poly'.
    The highest degree of the polynomial (1 for linear, 2 for quadratic, etc).


**```activation```** :&ensp;`{'relu', 'identity', 'logistic'` or `'tanh'}`, default `'relu'`
:   Only used if <code>method</code> = 'neural_net'.
    Activation function for the hidden layer, see sklearn.neural_network.MLPRegressor


**```max_iter```** :&ensp;<code>int</code>, default <code>500</code>
:   Only used if <code>method</code> = 'neural_net'.
    Maximum number of iterations.


**```solver```** :&ensp;`{'lbfgs', 'sgd', 'adam'}`, default `'lbfgs'`
:   Only used if <code>method</code> = 'neural_net'.
    The solver for weight optimization.


**```hidden_layer_sizes```** :&ensp;`array-like` of `shape(n_layers - 2,)`, default=<code>(10,)</code>
:   Only used if <code>method</code> = 'neural_net'.
    The ith element represents the number of neurons in the ith hidden layer.


**```alpha```** :&ensp;<code>float</code>, default `1e-16`
:   Only used if <code>method</code> = 'gmm'.
    Value of the covariance element of parameters.


**```fig```** :&ensp;<code>matplotlib.figure.Figure</code>, optional
:   figure to plot on. If None, then the new one is created.


**```show_fig```** :&ensp;<code>bool</code>, default <code>True</code>
:   If the figure will be shown.


**```return_fig```** :&ensp;<code>bool</code>, default <code>False</code>
:   If True, the figure and ax (or list of ax) will be returned.


**```**kwargs```**
:   other keyword arguments for <code>method</code> = 'neural_net', see sklearn.neural_network.MLPRegressor.

#### Returns

**```result```** :&ensp;<code>pandas.DataFrame</code>
:   Predicted table


**```fig```** :&ensp;<code>matplotlib.figure.Figure</code>
:   if <code>return\_fig</code> set to True


**```ax```** :&ensp;<code>matplotlib.axes.Axes</code> or <code>list</code> of <code>matplotlib.axes.Axes</code>
:   if <code>return\_fig</code> set to True


</details>
