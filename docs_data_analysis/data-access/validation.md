---
# Display h3 headings
toc_max_heading_level: 3
sidebar_label: 'Documentation'
description: 'Documentation'
hide_title: true
---



    
## Module `citros_data_analysis.validation` {#citros_data_analysis.validation}







    
## Class `Validation` {#citros_data_analysis.validation.Validation}





```python
class Validation(
    df=None,
    data_label=None,
    param_label=None,
    method='scale',
    num=100,
    units=''
)
```


<details>
  <summary>Description</summary>

Validation class.

#### Parameters

**```df```** :&ensp;**pandas.DataFrame**
:   Data table to perform validation tests on.


**```data_label```** :&ensp;**str** or **list** of **str**
:   Specifies label(s) of the data column(s) in data table.


**```param_label```** :&ensp;**str**
:   Label of column on the basis of which the indixes will be calculated.


**```method```** :&ensp;`{'scale', 'bin'}`, default `'scale'`
:   Method of data preparation: scaling to [0,1] interval or binning.


**```num```** :&ensp;**int**, default **100**
:   Number of points in a new scale that will be used for interpolation or number of bins.


**```units```** :&ensp;**str**, optional
:   Specifies units of the data.





</details>






    
### Method `mean_test` {#citros_data_analysis.validation.Validation.mean_test}




```python
def mean_test(
    self,
    limits=1.0
)
```


<details>
  <summary>Description</summary>

Test whether mean is inside the given limits.

#### Parameters

**```limits```** :&ensp;**float** or **list**, default **1.0**
:   Limit to test mean. If the data has multiple columns, limits may be set for each column by list. 
    That way list length must be equal to number of columns.

#### Returns

**```log```** :&ensp;**CitrosDict**
:   &nbsp;


**```table```** :&ensp;**pandas.DataFrame **
:   Table with results of the test.


**```fig```** :&ensp;**matplotlib.figure.Figure**
:   Figure with plotted simulations, mean values and limit boundaries.


</details>


    
### Method `norm_test` {#citros_data_analysis.validation.Validation.norm_test}




```python
def norm_test(
    self,
    norm_type='L2',
    limits=1.0
)
```


<details>
  <summary>Description</summary>

Test whether norm of the each simulation is less than the given limit.

#### Parameters

**```norm_type```** :&ensp;`{'L2', 'Linf'}`, default `'L2'`
:   Norm type.


**```limits```** :&ensp;**float** or **list**, default **1.0**
:   Limit to test simulation results. If the data has multiple columns, limits may be set for each column by list. 
    That way list length must be equal to number of columns.

#### Returns

**```log```** :&ensp;**CitrosDict**
:   &nbsp;


**```table```** :&ensp;**pandas.DataFrame **
:   Table with results of the test.


**```fig```** :&ensp;**matplotlib.figure.Figure**
:   Figure with plotted norm value and limits.


</details>


    
### Method `set_data_table` {#citros_data_analysis.validation.Validation.set_data_table}




```python
def set_data_table(
    self,
    df,
    data_label,
    param_label,
    method='scale',
    num=100,
    units=''
)
```


<details>
  <summary>Description</summary>

Set data table to perform validation tests on.

#### Parameters

**```df```** :&ensp;**pandas.DataFrame**
:   Data table to perform validation tests on.


**```data_label```** :&ensp;**str** or **list** of **str**
:   Specifies label(s) of the data column(s) in data table.


**```param_label```** :&ensp;**str**
:   Label of column on the basis of which the indixes will be calculated.


**```method```** :&ensp;`{'scale', 'bin'}`, default `'scale'`
:   Method of data preparation: scaling to [0,1] interval or binning.


**```num```** :&ensp;**int**, default **100**
:   Number of points in a new scale that will be used for interpolation or number of bins.


**```units```** :&ensp;**str**, optional
:   Specifies units of the data.


</details>


    
### Method `set_tests` {#citros_data_analysis.validation.Validation.set_tests}




```python
def set_tests(
    self,
    test_method={'std_bound': {'limits': 1.0, 'n_std': 3, 'nan_passed': True}, 'mean': {'limits': 1.0}, 'sid': {'limits': 1.0}, 'norm_L2': {'limits': 1.0}, 'norm_Linf': {'limits': 1.0}}
)
```


<details>
  <summary>Description</summary>

Perform tests on the data.

#### Parameters

**```tests_method```** :&ensp;**dict**
:   Keys define test methods and corresponding test parameters are stored as values.
    Possible test methods are:


<details>
    <summary>'std_bound'</summary>

Test whether standard deviation is inside the given limits.
Test parameters are stored as the dict with the following keys:
- 'limits' : float or list, default 1.0
    Limit to test standard deviation boundary. If the data has multiple columns, limits may be set for each column by list.  That way list length must be equal to number of columns.
- 'n_std' : int, default 3
    The parameter specifies the number of standard deviations to be within limits.
- 'nan_passed' : bool, default True
    If True, the nan values of standard deviation will pass the test.

</details>

<details>
    <summary>'mean'</summary>

Test whether mean is inside the given limits.
Test parameters are stored as the dict:
- 'limits' : float or list, default 1.0
    Limit to test mean. If the data has multiple columns, limits may be set for each column by list. 
    That way list length must be equal to number of columns.

</details>

<details>
    <summary>'sid'</summary>

Test whether all simulations are inside the given limits.
Test parameters are stored as the dict:
- 'limits' : float or list, default 1.0
    Limit to test simulation results. If the data has multiple columns, limits may be set for each column by list. 
    That way list length must be equal to number of columns.

</details>

<details>
    <summary>'norm_L2'</summary>

Test whether L2 norm of the each simulation is less than the given limit.
Test parameters are stored as the dict:
- 'limits' : float or list, default 1.0
    Limit to test simulation results. If the data has multiple columns, limits may be set for each column by list. 
    That way list length must be equal to number of columns.

</details>

<details>
    <summary>'norm_Linf'</summary>

Test whether Linf norm of the each simulation is less than the given limit.
Test parameters are stored as the dict:
- 'limits' : float or list, default 1.0
    Limit to test simulation results. If the data has multiple columns, limits may be set for each column by list. 
    That way list length must be equal to number of columns.

</details>

#### Returns

**```log```** :&ensp;**CitrosDict**
:   &nbsp;


**```tables```** :&ensp;**dict**
:   Dictionary with test methods as keys and pandas.DataFrame table with results of the test as values.


**```figures```** :&ensp;**dict**
:   Dictionary with test methods as keys and matplotlib.figure.Figure with test results as values.

#### See Also

**[Validation.std\_bound\_test()](#citros\_data\_analysis.validation.Validation.std\_bound\_test "citros\_data\_analysis.validation.Validation.std\_bound\_test")**, **[Validation.mean\_test()](#citros\_data\_analysis.validation.Validation.mean\_test "citros\_data\_analysis.validation.Validation.mean\_test")**, **[Validation.sid\_test()](#citros\_data\_analysis.validation.Validation.sid\_test "citros\_data\_analysis.validation.Validation.sid\_test")**, **[Validation.norm\_test()](#citros\_data\_analysis.validation.Validation.norm\_test "citros\_data\_analysis.validation.Validation.norm\_test")**

#### Examples

Test whether 3 standard deviation boundary spans inside [-0.3, 0.3] interval (treat nan values of the
standard deviation, if they are presented, as passed the test) and L2 norm of the each simulation is less than 12.5:

```python
>>> Validation.set_tests(test_method = {'std_bound' : {'limits' : 0.3, 'n_std': 3, 'nan_passed': True},
...                                     'norm_L2' : {'limits' : 12.5}})
```

</details>


    
### Method `sid_test` {#citros_data_analysis.validation.Validation.sid_test}




```python
def sid_test(
    self,
    limits=1.0
)
```


<details>
  <summary>Description</summary>

Test whether all simulations are inside the given limits.

#### Parameters

**```limits```** :&ensp;**float** or **list**, default **1.0**
:   Limit to test simulation results. If the data has multiple columns, limits may be set for each column by list. 
    That way list length must be equal to number of columns.

#### Returns

**```log```** :&ensp;**CitrosDict**
:   &nbsp;


**```table```** :&ensp;**pandas.DataFrame **
:   Table with results of the test.


**```fig```** :&ensp;**matplotlib.figure.Figure**
:   Figure with plotted simulations, mean values and limit boundaries.


</details>


    
### Method `std_bound_test` {#citros_data_analysis.validation.Validation.std_bound_test}




```python
def std_bound_test(
    self,
    limits=1.0,
    n_std=3,
    nan_passed=True
)
```


<details>
  <summary>Description</summary>

Test whether **n\_std**-standard deviation boundary is inside the given limits.

#### Parameters

**```limits```** :&ensp;**float** or **list**, default **1.0**
:   Limit to test standard deviation boundary. If the data has multiple columns, limits may be set for each column by list. 
    That way list length must be equal to number of columns.


**```n_std```** :&ensp;**int**, default **3**
:   The parameter specifies the number of standard deviations to be within limits.


**```nan_passed```** :&ensp;**bool**, default **True**
:   If True, the nan values of standard deviation will pass the test.

#### Returns

**```log```** :&ensp;**CitrosDict**
:   &nbsp;


**```table```** :&ensp;```pandas.DataFrame```
:   Table with results of the test.


**```fig```** :&ensp;**matplotlib.figure.Figure**
:   Figure with plotted simulations, mean values, standard deviation boundaries and limit boundaries.

#### Examples

Test whether 3 standard deviation boundary spans inside [-0.3, 0.3] interval (treat nan values of the
standard deviation, if they are presented, as passed the test) and L2 norm of the each simulation is less than 12.5:

```python
>>> Validation.set_tests(test_method = {'std_bound' : {'limits' : 0.3, 'n_std': 3, 'nan_passed': True},
...                                     'norm_L2' : {'limits' : 12.5}})
```

</details>
