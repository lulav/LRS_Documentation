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
    component: ComponentCreator('/blog', 'a61'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', 'b93'),
    exact: true
  },
  {
    path: '/blog/first-blog-post',
    component: ComponentCreator('/blog/first-blog-post', '175'),
    exact: true
  },
  {
    path: '/blog/first-blog-post-example',
    component: ComponentCreator('/blog/first-blog-post-example', 'd4c'),
    exact: true
  },
  {
    path: '/blog/long-blog-post',
    component: ComponentCreator('/blog/long-blog-post', '07b'),
    exact: true
  },
  {
    path: '/blog/mdx-blog-post',
    component: ComponentCreator('/blog/mdx-blog-post', '123'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', 'de3'),
    exact: true
  },
  {
    path: '/blog/tags/docusaurus',
    component: ComponentCreator('/blog/tags/docusaurus', '696'),
    exact: true
  },
  {
    path: '/blog/tags/facebook',
    component: ComponentCreator('/blog/tags/facebook', 'cb6'),
    exact: true
  },
  {
    path: '/blog/tags/greetings',
    component: ComponentCreator('/blog/tags/greetings', '788'),
    exact: true
  },
  {
    path: '/blog/tags/hello',
    component: ComponentCreator('/blog/tags/hello', '49b'),
    exact: true
  },
  {
    path: '/blog/tags/hola',
    component: ComponentCreator('/blog/tags/hola', '576'),
    exact: true
  },
  {
    path: '/blog/welcome',
    component: ComponentCreator('/blog/welcome', 'be5'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '883'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'ea4'),
    routes: [
      {
        path: '/docs/intro',
        component: ComponentCreator('/docs/intro', '553'),
        exact: true,
        sidebar: "gettingStartedSidebar"
      }
    ]
  },
  {
    path: '/docs-data-analysis',
    component: ComponentCreator('/docs-data-analysis', '41e'),
    routes: [
      {
        path: '/docs-data-analysis/category/data-access',
        component: ComponentCreator('/docs-data-analysis/category/data-access', 'f0f'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/category/error-analysis',
        component: ComponentCreator('/docs-data-analysis/category/error-analysis', 'a23'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/category/modules',
        component: ComponentCreator('/docs-data-analysis/category/modules', 'f01'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/intro',
        component: ComponentCreator('/docs-data-analysis/intro', '146'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/modules/data-access/data_access_description',
        component: ComponentCreator('/docs-data-analysis/modules/data-access/data_access_description', '8cf'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/modules/data-access/data_access_examples',
        component: ComponentCreator('/docs-data-analysis/modules/data-access/data_access_examples', '030'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/modules/error-analysis/error_analysis_description',
        component: ComponentCreator('/docs-data-analysis/modules/error-analysis/error_analysis_description', 'e59'),
        exact: true,
        sidebar: "dataAnalysisdSidebar"
      },
      {
        path: '/docs-data-analysis/modules/error-analysis/error_analysis_examples',
        component: ComponentCreator('/docs-data-analysis/modules/error-analysis/error_analysis_examples', '740'),
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
