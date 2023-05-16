import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '4af'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '366'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'c62'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', '26c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '8a9'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', 'e82'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', 'f2f'),
    exact: true
  },
  {
    path: '/blog',
    component: ComponentCreator('/blog', '4c1'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', 'b93'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', 'de3'),
    exact: true
  },
  {
    path: '/blog/tags/greetings',
    component: ComponentCreator('/blog/tags/greetings', '027'),
    exact: true
  },
  {
    path: '/blog/welcome',
    component: ComponentCreator('/blog/welcome', 'a08'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '883'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'e68'),
    routes: [
      {
        path: '/docs/',
        component: ComponentCreator('/docs/', 'b8b'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs/new_page',
        component: ComponentCreator('/docs/new_page', '5d3'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      }
    ]
  },
  {
    path: '/docs-citros',
    component: ComponentCreator('/docs-citros', 'cda'),
    routes: [
      {
        path: '/docs-citros/',
        component: ComponentCreator('/docs-citros/', 'aa8'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-citros/run_in_cloud',
        component: ComponentCreator('/docs-citros/run_in_cloud', '448'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-citros/run_locally',
        component: ComponentCreator('/docs-citros/run_locally', 'c08'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-citros/simulation_run',
        component: ComponentCreator('/docs-citros/simulation_run', 'f76'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      }
    ]
  },
  {
    path: '/docs-cli',
    component: ComponentCreator('/docs-cli', 'e3e'),
    routes: [
      {
        path: '/docs-cli/',
        component: ComponentCreator('/docs-cli/', '62e'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      }
    ]
  },
  {
    path: '/docs-data-analysis',
    component: ComponentCreator('/docs-data-analysis', 'fb2'),
    routes: [
      {
        path: '/docs-data-analysis/',
        component: ComponentCreator('/docs-data-analysis/', '6c2'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/category/data-access',
        component: ComponentCreator('/docs-data-analysis/category/data-access', '2bd'),
        exact: true,
        sidebar: "gettingStartedSidebar"
      },
      {
        path: '/docs-data-analysis/category/data-access-1',
        component: ComponentCreator('/docs-data-analysis/category/data-access-1', 'b20'),
        exact: true,
        sidebar: "citrosdSidebar"
      },
      {
        path: '/docs-data-analysis/category/data-access-2',
        component: ComponentCreator('/docs-data-analysis/category/data-access-2', '10a'),
        exact: true,
        sidebar: "clidSidebar"
      },
      {
        path: '/docs-data-analysis/category/data-access-3',
        component: ComponentCreator('/docs-data-analysis/category/data-access-3', '700'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/category/error-analysis',
        component: ComponentCreator('/docs-data-analysis/category/error-analysis', 'bfe'),
        exact: true,
        sidebar: "gettingStartedSidebar"
      },
      {
        path: '/docs-data-analysis/category/error-analysis-1',
        component: ComponentCreator('/docs-data-analysis/category/error-analysis-1', '927'),
        exact: true,
        sidebar: "citrosdSidebar"
      },
      {
        path: '/docs-data-analysis/category/error-analysis-2',
        component: ComponentCreator('/docs-data-analysis/category/error-analysis-2', 'afe'),
        exact: true,
        sidebar: "clidSidebar"
      },
      {
        path: '/docs-data-analysis/category/error-analysis-3',
        component: ComponentCreator('/docs-data-analysis/category/error-analysis-3', '7cf'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/data-access/data_access_description',
        component: ComponentCreator('/docs-data-analysis/data-access/data_access_description', 'e80'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/data-access/data_access_examples',
        component: ComponentCreator('/docs-data-analysis/data-access/data_access_examples', '47d'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/error-analysis/error_analysis_description',
        component: ComponentCreator('/docs-data-analysis/error-analysis/error_analysis_description', '8de'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/error-analysis/error_analysis_examples',
        component: ComponentCreator('/docs-data-analysis/error-analysis/error_analysis_examples', 'd81'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '066'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
