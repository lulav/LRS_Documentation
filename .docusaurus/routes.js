import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/doc/__docusaurus/debug',
    component: ComponentCreator('/doc/__docusaurus/debug', 'da8'),
    exact: true
  },
  {
    path: '/doc/__docusaurus/debug/config',
    component: ComponentCreator('/doc/__docusaurus/debug/config', 'a12'),
    exact: true
  },
  {
    path: '/doc/__docusaurus/debug/content',
    component: ComponentCreator('/doc/__docusaurus/debug/content', 'f7c'),
    exact: true
  },
  {
    path: '/doc/__docusaurus/debug/globalData',
    component: ComponentCreator('/doc/__docusaurus/debug/globalData', '3d5'),
    exact: true
  },
  {
    path: '/doc/__docusaurus/debug/metadata',
    component: ComponentCreator('/doc/__docusaurus/debug/metadata', '1be'),
    exact: true
  },
  {
    path: '/doc/__docusaurus/debug/registry',
    component: ComponentCreator('/doc/__docusaurus/debug/registry', '1ec'),
    exact: true
  },
  {
    path: '/doc/__docusaurus/debug/routes',
    component: ComponentCreator('/doc/__docusaurus/debug/routes', '015'),
    exact: true
  },
  {
    path: '/doc/blog',
    component: ComponentCreator('/doc/blog', 'a10'),
    exact: true
  },
  {
    path: '/doc/blog/archive',
    component: ComponentCreator('/doc/blog/archive', '0d0'),
    exact: true
  },
  {
    path: '/doc/blog/tags',
    component: ComponentCreator('/doc/blog/tags', '076'),
    exact: true
  },
  {
    path: '/doc/blog/tags/greetings',
    component: ComponentCreator('/doc/blog/tags/greetings', '662'),
    exact: true
  },
  {
    path: '/doc/blog/welcome',
    component: ComponentCreator('/doc/blog/welcome', '60e'),
    exact: true
  },
  {
    path: '/doc/markdown-page',
    component: ComponentCreator('/doc/markdown-page', 'e5e'),
    exact: true
  },
  {
    path: '/doc/docs',
    component: ComponentCreator('/doc/docs', '953'),
    routes: [
      {
        path: '/doc/docs/',
        component: ComponentCreator('/doc/docs/', '0fc'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/doc/docs/new_page',
        component: ComponentCreator('/doc/docs/new_page', '73b'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      }
    ]
  },
  {
    path: '/doc/docs-citros',
    component: ComponentCreator('/doc/docs-citros', 'fba'),
    routes: [
      {
        path: '/doc/docs-citros/',
        component: ComponentCreator('/doc/docs-citros/', '991'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/doc/docs-citros/run_in_cloud',
        component: ComponentCreator('/doc/docs-citros/run_in_cloud', '94a'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/doc/docs-citros/run_locally',
        component: ComponentCreator('/doc/docs-citros/run_locally', '564'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/doc/docs-citros/simulation_run',
        component: ComponentCreator('/doc/docs-citros/simulation_run', '6c8'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      }
    ]
  },
  {
    path: '/doc/docs-cli',
    component: ComponentCreator('/doc/docs-cli', '909'),
    routes: [
      {
        path: '/doc/docs-cli/',
        component: ComponentCreator('/doc/docs-cli/', 'b08'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      }
    ]
  },
  {
    path: '/doc/docs-data-analysis',
    component: ComponentCreator('/doc/docs-data-analysis', '977'),
    routes: [
      {
        path: '/doc/docs-data-analysis/',
        component: ComponentCreator('/doc/docs-data-analysis/', '002'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/doc/docs-data-analysis/category/data-access',
        component: ComponentCreator('/doc/docs-data-analysis/category/data-access', 'b3e'),
        exact: true,
        sidebar: "gettingStartedSidebar"
      },
      {
        path: '/doc/docs-data-analysis/category/data-access-1',
        component: ComponentCreator('/doc/docs-data-analysis/category/data-access-1', '50c'),
        exact: true,
        sidebar: "citrosdSidebar"
      },
      {
        path: '/doc/docs-data-analysis/category/data-access-2',
        component: ComponentCreator('/doc/docs-data-analysis/category/data-access-2', '3d2'),
        exact: true,
        sidebar: "clidSidebar"
      },
      {
        path: '/doc/docs-data-analysis/category/data-access-3',
        component: ComponentCreator('/doc/docs-data-analysis/category/data-access-3', '636'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/doc/docs-data-analysis/category/error-analysis',
        component: ComponentCreator('/doc/docs-data-analysis/category/error-analysis', '068'),
        exact: true,
        sidebar: "gettingStartedSidebar"
      },
      {
        path: '/doc/docs-data-analysis/category/error-analysis-1',
        component: ComponentCreator('/doc/docs-data-analysis/category/error-analysis-1', '2fa'),
        exact: true,
        sidebar: "citrosdSidebar"
      },
      {
        path: '/doc/docs-data-analysis/category/error-analysis-2',
        component: ComponentCreator('/doc/docs-data-analysis/category/error-analysis-2', '600'),
        exact: true,
        sidebar: "clidSidebar"
      },
      {
        path: '/doc/docs-data-analysis/category/error-analysis-3',
        component: ComponentCreator('/doc/docs-data-analysis/category/error-analysis-3', '97a'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/doc/docs-data-analysis/data-access/data_access_description',
        component: ComponentCreator('/doc/docs-data-analysis/data-access/data_access_description', '16e'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/doc/docs-data-analysis/data-access/data_access_examples',
        component: ComponentCreator('/doc/docs-data-analysis/data-access/data_access_examples', 'aed'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/doc/docs-data-analysis/error-analysis/error_analysis_description',
        component: ComponentCreator('/doc/docs-data-analysis/error-analysis/error_analysis_description', '3b7'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/doc/docs-data-analysis/error-analysis/error_analysis_examples',
        component: ComponentCreator('/doc/docs-data-analysis/error-analysis/error_analysis_examples', '054'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      }
    ]
  },
  {
    path: '/doc/',
    component: ComponentCreator('/doc/', 'b75'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
