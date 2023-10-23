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


module.exports = {
  gettingStartedSidebar: [
    {
      type: "category",
      label: "Quick Start",
      // link: {
      //   // type: "doc",
      //   id: "index",
      // },
      items: [
        {
          type: "doc",
          id: "quickstart/quick_tbd",
          label: "CITROS web",
        },
      ],
    },

    // //Notifications
    // {
    //   type: 'category',
    //   label: 'Notifications',
    //   collapsible: true,
    //   collapsed: true,
    //   items: [
    //     {
    //       type: 'doc',
    //       label: 'Notification Template',
    //       id: 'notifications/notifications_template',
    //     },
    //     {
    //       type: 'doc',
    //       label: 'Notifications Settings',
    //       id: 'notifications/notifications_settings',
    //     },
    //   ],
    // },



    // //CI/CD and DevOps
    // {
    //   type: 'category',
    //   label: 'CI/CD and DevOps',
    //   collapsible: true,
    //   collapsed: true,
    //   items: [
    //     {
    //       type: 'doc',
    //       label: 'TBD',
    //       id: 'cicd/cicd_tbd',
    //     },
    //   ],
    // }
  ]
};