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
    component: ComponentCreator('/doc/blog', 'ad0'),
    exact: true
  },
  {
    path: '/doc/blog/2023/04/02/citros-first',
    component: ComponentCreator('/doc/blog/2023/04/02/citros-first', '459'),
    exact: true
  },
  {
    path: '/doc/blog/archive',
    component: ComponentCreator('/doc/blog/archive', '0d0'),
    exact: true
  },
  {
    path: '/doc/blog/first-blog-post',
    component: ComponentCreator('/doc/blog/first-blog-post', '26a'),
    exact: true
  },
  {
    path: '/doc/blog/long-blog-post',
    component: ComponentCreator('/doc/blog/long-blog-post', '1c6'),
    exact: true
  },
  {
    path: '/doc/blog/mdx-blog-post',
    component: ComponentCreator('/doc/blog/mdx-blog-post', 'ab9'),
    exact: true
  },
  {
    path: '/doc/blog/tags',
    component: ComponentCreator('/doc/blog/tags', '076'),
    exact: true
  },
  {
    path: '/doc/blog/tags/docusaurus',
    component: ComponentCreator('/doc/blog/tags/docusaurus', '9c9'),
    exact: true
  },
  {
    path: '/doc/blog/tags/facebook',
    component: ComponentCreator('/doc/blog/tags/facebook', 'a42'),
    exact: true
  },
  {
    path: '/doc/blog/tags/hello',
    component: ComponentCreator('/doc/blog/tags/hello', '9e0'),
    exact: true
  },
  {
    path: '/doc/blog/tags/hola',
    component: ComponentCreator('/doc/blog/tags/hola', '3db'),
    exact: true
  },
  {
    path: '/doc/blog/welcome',
    component: ComponentCreator('/doc/blog/welcome', '8d9'),
    exact: true
  },
  {
    path: '/doc/markdown-page',
    component: ComponentCreator('/doc/markdown-page', 'e5e'),
    exact: true
  },
  {
    path: '/doc/docs',
    component: ComponentCreator('/doc/docs', '18c'),
    routes: [
      {
        path: '/doc/docs/category/tutorial---basics',
        component: ComponentCreator('/doc/docs/category/tutorial---basics', 'f13'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/category/tutorial---extras',
        component: ComponentCreator('/doc/docs/category/tutorial---extras', '953'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/intro',
        component: ComponentCreator('/doc/docs/intro', '464'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/tests1/test',
        component: ComponentCreator('/doc/docs/tests1/test', '721'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/tests1/test2',
        component: ComponentCreator('/doc/docs/tests1/test2', '95c'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/tutorial-basics/congratulations',
        component: ComponentCreator('/doc/docs/tutorial-basics/congratulations', '11a'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/tutorial-basics/create-a-blog-post',
        component: ComponentCreator('/doc/docs/tutorial-basics/create-a-blog-post', '088'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/tutorial-basics/create-a-document',
        component: ComponentCreator('/doc/docs/tutorial-basics/create-a-document', 'fc4'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/tutorial-basics/create-a-page',
        component: ComponentCreator('/doc/docs/tutorial-basics/create-a-page', '21e'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/tutorial-basics/deploy-your-site',
        component: ComponentCreator('/doc/docs/tutorial-basics/deploy-your-site', '3fe'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/tutorial-basics/markdown-features',
        component: ComponentCreator('/doc/docs/tutorial-basics/markdown-features', '84f'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/tutorial-extras/manage-docs-versions',
        component: ComponentCreator('/doc/docs/tutorial-extras/manage-docs-versions', '125'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/doc/docs/tutorial-extras/translate-your-site',
        component: ComponentCreator('/doc/docs/tutorial-extras/translate-your-site', '934'),
        exact: true,
        sidebar: "tutorialSidebar"
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
