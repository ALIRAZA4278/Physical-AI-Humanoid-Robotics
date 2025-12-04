// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const { themes } = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From ROS 2 to Voice-Controlled Autonomous Humanoids',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://aliraza4278.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/Physical-AI-Humanoid-Robotics/',

  // GitHub pages deployment config
  organizationName: 'ALIRAZA4278',
  projectName: 'Physical-AI-Humanoid-Robotics',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Add custom head tags
  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossorigin: 'anonymous',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'stylesheet',
        href: 'https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&family=JetBrains+Mono:wght@400;500&display=swap',
      },
    },
  ],

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/ALIRAZA4278/Physical-AI-Humanoid-Robotics/edit/main/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Color mode configuration
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },

      // Announcement bar
      announcementBar: {
        id: 'beta_notice',
        content:
          '🚀 <b>Module 1 Now Available!</b> Start learning ROS 2 Fundamentals today.',
        backgroundColor: 'linear-gradient(135deg, #0891b2, #7c3aed)',
        textColor: '#ffffff',
        isCloseable: true,
      },

      // Social card image
      image: 'img/social-card.png',

      // Navbar configuration
      navbar: {
        title: '',
        hideOnScroll: false,
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
          srcDark: 'img/logo.svg',
          width: 120,
          height: 40,
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Documentation',
          },
          {
            to: '/docs/module-1-ros2/architecture',
            label: 'ROS 2',
            position: 'left',
          },
          {
            to: '/docs/module-3-isaac/isaac-setup',
            label: 'Isaac AI',
            position: 'left',
          },
          {
            to: '/docs/appendix/hardware-requirements',
            label: 'Hardware',
            position: 'left',
          },
          {
            href: 'https://github.com/ALIRAZA4278/Physical-AI-Humanoid-Robotics',
            position: 'right',
            className: 'header-github-link',
            'aria-label': 'GitHub repository',
          },
        ],
      },

      // Footer configuration
      footer: {
        style: 'light',
        links: [
          {
            title: 'Learn',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Module 1: ROS 2',
                to: '/docs/module-1-ros2/architecture',
              },
              {
                label: 'Module 2: Simulation',
                to: '/docs/module-2-simulation/gazebo-setup',
              },
              {
                label: 'Module 3: Isaac AI',
                to: '/docs/module-3-isaac/isaac-setup',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Hardware Requirements',
                to: '/docs/appendix/hardware-requirements',
              },
              {
                label: 'Version Migration',
                to: '/docs/appendix/version-migration',
              },
              {
                label: 'Troubleshooting',
                to: '/docs/appendix/troubleshooting',
              },
              {
                label: 'Glossary',
                to: '/docs/glossary',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'ROS Discourse',
                href: 'https://discourse.ros.org/',
              },
              {
                label: 'NVIDIA Isaac Forum',
                href: 'https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/',
              },
              {
                label: 'Gazebo Community',
                href: 'https://community.gazebosim.org/',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/ALIRAZA4278/Physical-AI-Humanoid-Robotics',
              },
              {
                label: 'ROS 2 Docs',
                href: 'https://docs.ros.org/en/jazzy/',
              },
              {
                label: 'Isaac Sim Docs',
                href: 'https://docs.isaacsim.omniverse.nvidia.com/',
              },
            ],
          },
        ],
        copyright: `© ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
      },

      // Prism code highlighting
      prism: {
        theme: themes.github,
        darkTheme: themes.dracula,
        additionalLanguages: ['python', 'bash', 'yaml', 'json', 'markup'],
        defaultLanguage: 'python',
        magicComments: [
          {
            className: 'theme-code-block-highlighted-line',
            line: 'highlight-next-line',
            block: { start: 'highlight-start', end: 'highlight-end' },
          },
        ],
      },

      // Table of contents
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 4,
      },
    }),
};

module.exports = config;
