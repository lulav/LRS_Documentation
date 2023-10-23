---
# Display h3 headings
sidebar_label: 'class Ref'
toc_max_heading_level: 3
hide_title: true
description: 'Documentation'
---








    
## Class Ref {#citros_data_analysis.data_access.references.Ref}





```python
class Ref
```


<details>
  <summary>Description</summary>

Stores references to the batches used in the work



</details>






    
### Method print {#citros_data_analysis.data_access.references.Ref.print}




```python
def print()
```


<details>
  <summary>Description</summary>

Print the information about all batches that were used.

Displays the batch creator's first and last name and email, batch name, message and creation time, link to the batch.
The output is sorted by the last names.

---
#### Examples

Display references to the batches that were used in the current notebook:

```python
>>> from citros_data_analysis import data_access as da
>>> ref = da.Ref()
>>> ref.print()
stevenson mary, mary@mail.com
robotics, 'robotics system', 2023-06-01 09:00:00
<https://citros.io/robot_master/batch/00000000-aaaa-1111-2222-333333333333/>
```

</details>