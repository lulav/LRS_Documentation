---
sidebar_position: 1
sidebar_label: 'Package overview'
hide_title: true
---

## Package overview

This package is a comprehensive solution for data query, analysis and visualization. With its extensive features, you can quickly and easily extract valuable insights from your data.

The package is devided into two modules: [Data access](#data-access) and [Error analysis](#error-analysis).

### Data access

The first module, [**data_access**](data-access/data_access_description), is designed to communicate with the database and obtain information about the selected data packages.
You will be able to effortlessly get overview about the simulated data and its structure, create queries and download the data you are interested in. Once you have your data, the package provides set of tools for data visualisation and manipulation, making it easy to prepare your data for future analysis. See the [examples](data-access/data_access_examples) of its main features.


import ThemedImage from '@theme/ThemedImage';
import useBaseUrl from '@docusaurus/useBaseUrl';

<ThemedImage
  alt="Docusaurus themed image"
  sources={{
    light: useBaseUrl('/img/img_data_access_light.png'),
    dark: useBaseUrl('/img/img_data_access_dark.png'),
  }}
/>

### Error analysis

Second module [**error_analysis**](error-analysis/error_analysis_description) containes a range of analysis methods, including statistical analysis and regressions, enabling you to visualize the results and derive valuable insights from your data. It provides an easy way to check statistics, explore correlations, and make predictions using regressions. See the [examples](error-analysis/error_analysis_examples) about the module usage.

<ThemedImage
  alt="Docusaurus themed image"
  sources={{
    light: useBaseUrl('/img/img_error_analysis_light.png'),
    dark: useBaseUrl('/img/img_error_analysis_dark.png'),
  }}
/>;
