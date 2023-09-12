/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

// /** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
// const sidebars = {
//   // By default, Docusaurus generates a sidebar from the docs folder structure
//   gettingStartedSidebar: [{type: 'autogenerated', dirName: '.'}],
// };

// module.exports = sidebars;

// module.exports = {
//   gettingStartedSidebar: [
//     { type: "doc", id: "index" },
//     {
//       type: 'doc',
//       id: 'citros_web',
//       label: 'CITROS web',
//     },
//     {
//       type: 'doc',
//       id: 'cli',
//       label: 'CLI',
//     },
//     {
//       type: 'doc',
//       id: 'data_analysis',
//       label: 'Data analysis',
//     },
//   ],
// };

module.exports = {
  gettingStartedSidebar: [
    {
      type: "category",
      label: "Getting started",
      link: {
        type: "doc",
        id: "index",
      },
      items: [
        {
          type: "doc",
          id: "citros_web",
          label: "CITROS web",
          // image: '/static/img/citros_web_light.png',
        },
        {
          type: "doc",
          id: "cli",
          label: "CLI",
          // image: '/static/img/citros_cli.png',
        },
        {
          type: "doc",
          id: "data_analysis",
          label: "Data analysis",
        },
      ],
    },
  ],
};
