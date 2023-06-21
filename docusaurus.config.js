// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

// const lightCodeTheme = require('prism-react-renderer/themes/github');
// const darkCodeTheme = require('prism-react-renderer/themes/dracula');
const lightCodeTheme = require('prism-react-renderer/themes/vsLight');
const darkCodeTheme = require('prism-react-renderer/themes/vsDark');
const math = require('remark-math');
const katex = require('rehype-katex');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'CITROS',
  tagline: 'The starting point for your next robotic project',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  // url: 'https://citros.io',
  url: 'http://localhost:3000',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/doc/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'citros', // Usually your GitHub org/user name.
  projectName: 'citros', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Matkdown
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          //routeBasePath: 'docs',
          path: 'docs',
          sidebarPath: require.resolve('./sidebars.js'),
          remarkPlugins: [math],
          rehypePlugins: [katex],
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          //editUrl:
          //  'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          //editUrl:
          //  'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs_citros_web',
        path: 'docs_citros_web',
        routeBasePath: 'docs_citros_web',
        sidebarPath: require.resolve('./sidebarsCitros.js'),
        remarkPlugins: [math],
        rehypePlugins: [katex],
        // includeCurrentVersion: false,
      }, 
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs_cli',
        path: 'docs_cli',
        routeBasePath: 'docs_cli',
        sidebarPath: require.resolve('./sidebarsCLI.js'),
        remarkPlugins: [math],
        rehypePlugins: [katex],
        // includeCurrentVersion: false,
      }, 
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs_data_analysis',
        path: 'docs_data_analysis',
        routeBasePath: 'docs_data_analysis',
        sidebarPath: require.resolve('./sidebarsDataAnalysis.js'),
        remarkPlugins: [math],
        rehypePlugins: [katex],
        // includeCurrentVersion: false,
      }, 
    ],
  ],

  stylesheets: [
    {
      href: 'katex/katex.min.css',
      type: 'text/css',
      // href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
      // type: 'text/css',
      // integrity:
      //   'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
      // crossorigin: 'anonymous',
    },
  ],
  
  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      
      navbar: {
        // title: 'CITROS',
        logo: {
          alt: 'CITROS Logo',
          // src: 'img/logo.svg',
          src: 'img/citros.png',
        },
        items: [
          {
            //to: '/docs/intro',    // ./docs/Intro.md
            type: 'docSidebar',
            sidebarId: 'gettingStartedSidebar',
            label: 'Getting started',
            position: 'left',
            activeBaseRegex: `/docs/`,
          },
          {
            to: '/docs_citros_web',    // ./docs-api/Intro.md
            label: 'Web',
            position: 'left',
            activeBaseRegex: `/docs_citros_web/`,
          },
          {
            to: '/docs_cli',    // ./docs-api/Intro.md
            label: 'CLI',
            position: 'left',
            activeBaseRegex: `/docs_cli/`,
            // type: 'docsVersionDropdown',
            // docsPluginId: 'docs_cli'
          },
          {
            to: '/docs_data_analysis',    // ./docs-api/Intro.md
            label: 'Data analysis',
            position: 'left',
            activeBaseRegex: `/docs_data_analysis/`,
          },
          {to: '/blog', label: 'Blog', position: 'left' },
          {
            type: 'docsVersionDropdown',
            position: 'right',
            docsPluginId: 'docs_cli',
            // dropdownItemsAfter: [{to: '/versions', label: 'All versions'}],
            // dropdownActiveClassDisabled: true,
          },
          {
            type: 'docsVersionDropdown',
            position: 'right',
            docsPluginId: 'docs_citros_web',
            // dropdownItemsAfter: [{to: '/versions', label: 'All versions'}],
            // dropdownActiveClassDisabled: true,
          },
          {
            type: 'docsVersionDropdown',
            position: 'right',
            docsPluginId: 'docs_data_analysis',
            // dropdownItemsAfter: [{to: '/versions', label: 'All versions'}],
            dropdownActiveClassDisabled: true,
          },
        ],
      },
      // footer: {
      //   style: 'dark',
      //   links: [
      //     {
      //       title: 'Tutorials',
      //       items: [
      //         {
      //           label: 'Getting started',
      //           to: '/docs',
      //         },
      //         {
      //           label: 'Web',
      //           to: '/docs_citros_web',
      //         },
      //         {
      //           label: 'CLI',
      //           to: '/docs_cli',
      //         },
      //         {
      //           label: 'Data analysis',
      //           to: '/docs_data_analysis',
      //         },
      //       ],
      //     },
      //     {
      //       title: 'More',
      //       items: [
      //         {
      //           label: 'Blog',
      //           to: '/blog',
      //         },
      //       ],
      //     },
      //   ],
      //   copyright: `Copyright © ${new Date().getFullYear()} My Project, Inc. Built with Docusaurus.`,
      // },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python'],
      },
    }),
};

module.exports = config;
