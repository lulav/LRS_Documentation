// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

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
        id: 'docs-citros',
        path: 'docs-citros',
        routeBasePath: 'docs-citros',
        sidebarPath: require.resolve('./sidebarsCitros.js'),
        // includeCurrentVersion: false,
      }, 
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-cli',
        path: 'docs-cli',
        routeBasePath: 'docs-cli',
        sidebarPath: require.resolve('./sidebarsCLI.js'),
        // includeCurrentVersion: false,
      }, 
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-data-analysis',
        path: 'docs-data-analysis',
        routeBasePath: 'docs-data-analysis',
        sidebarPath: require.resolve('./sidebarsDataAnalysis.js'),
        // includeCurrentVersion: false,
      }, 
    ],
  ],

  stylesheets: [
    {
      href: '/katex/katex.min.css',
      type: 'text/css',
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
            to: '/docs-citros',    // ./docs-api/Intro.md
            label: 'Web',
            position: 'left',
            activeBaseRegex: `/docs-citros/`,
          },
          {
            to: '/docs-cli',    // ./docs-api/Intro.md
            label: 'CLI',
            position: 'left',
            activeBaseRegex: `/docs-cli/`,
            // type: 'docsVersionDropdown',
            // docsPluginId: 'docs-cli'
          },
          {
            to: '/docs-data-analysis',    // ./docs-api/Intro.md
            label: 'Data analysis',
            position: 'left',
            activeBaseRegex: `/docs-data-analysis/`,
          },
          {to: '/blog', label: 'Blog', position: 'left' },
          {
            type: 'docsVersionDropdown',
            position: 'right',
            docsPluginId: 'docs-cli',
            // dropdownItemsAfter: [{to: '/versions', label: 'All versions'}],
            // dropdownActiveClassDisabled: true,
          },
          {
            type: 'docsVersionDropdown',
            position: 'right',
            docsPluginId: 'docs-citros',
            // dropdownItemsAfter: [{to: '/versions', label: 'All versions'}],
            // dropdownActiveClassDisabled: true,
          },
          {
            type: 'docsVersionDropdown',
            position: 'right',
            docsPluginId: 'docs-data-analysis',
            // dropdownItemsAfter: [{to: '/versions', label: 'All versions'}],
            // dropdownActiveClassDisabled: true,
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Tutorials',
            items: [
              {
                label: 'Getting started',
                to: '/docs',
              },
              {
                label: 'Web',
                to: '/docs-citros',
              },
              {
                label: 'CLI',
                to: '/docs-cli',
              },
              {
                label: 'Data analysis',
                to: '/docs-data-analysis',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Blog',
                to: '/blog',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} My Project, Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;
