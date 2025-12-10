import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Comprehensive Guide to Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-username.github.io', // GitHub Pages URL format
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Physical_AI_Humanoid_Robotics/', // Project name for GitHub Pages

  // GitHub pages deployment config.
  organizationName: 'your-username', // Usually your GitHub org/user name.
  projectName: 'Physical_AI_Humanoid_Robotics', // Usually your repo name.
  trailingSlash: false, // Recommended for GitHub Pages

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // i18n configuration
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // SEO and meta tags
  headTags: [
    // Preconnect to external resources for faster loading
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
    // DNS prefetch for performance
    {
      tagName: 'link',
      attributes: {
        rel: 'dns-prefetch',
        href: 'https://github.com',
      },
    },
    // Theme color for mobile browsers
    {
      tagName: 'meta',
      attributes: {
        name: 'theme-color',
        content: '#2e8555',
      },
    },
    // Mobile optimization
    {
      tagName: 'meta',
      attributes: {
        name: 'viewport',
        content: 'width=device-width, initial-scale=1, maximum-scale=5',
      },
    },
  ],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/your-username/Physical_AI_Humanoid_Robotics/tree/main/website/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
            blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
        sitemap: {
          changefreq: 'weekly',
          priority: 0.5,
          ignorePatterns: ['/tags/**'],
          filename: 'sitemap.xml',
        },
        gtag: undefined, // Disabled to reduce bundle size
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    // Image optimization plugin
    [
      '@docusaurus/plugin-ideal-image',
      {
        quality: 80,
        max: 1030,
        min: 640,
        steps: 2,
        disableInDev: false,
      },
    ],
  ],

  themeConfig: {
    // SEO - Social card for link previews
    image: 'img/physical-ai-social-card.jpg',

    // SEO metadata
    metadata: [
      {name: 'keywords', content: 'physical AI, humanoid robotics, ROS2, Isaac Sim, machine learning, robotics tutorial'},
      {name: 'author', content: 'Physical AI Book Team'},
      {name: 'robots', content: 'index, follow'},
      {property: 'og:type', content: 'website'},
      {property: 'og:title', content: 'Physical AI & Humanoid Robotics'},
      {property: 'og:description', content: 'Comprehensive guide to Physical AI and Humanoid Robotics with hands-on tutorials'},
      {name: 'twitter:card', content: 'summary_large_image'},
      {name: 'twitter:title', content: 'Physical AI & Humanoid Robotics'},
      {name: 'twitter:description', content: 'Learn Physical AI and Humanoid Robotics with comprehensive tutorials'},
    ],

    // Color mode settings
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },

    // Announcement bar (optional, can be removed)
    announcementBar: {
      id: 'support_us',
      content: 'Welcome to the Physical AI & Humanoid Robotics comprehensive guide!',
      backgroundColor: '#2e8555',
      textColor: '#fff',
      isCloseable: true,
    },

    navbar: {
      title: 'Physical AI & Robotics',
      hideOnScroll: true, // Better mobile UX
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/your-username/Physical_AI_Humanoid_Robotics',
          label: 'GitHub',
          position: 'right',
          'aria-label': 'GitHub repository',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book',
          items: [
            {
              label: 'Introduction',
              to: '/docs/introduction/what-is-physical-ai',
            },
            {
              label: 'ROS2 Fundamentals',
              to: '/docs/ros2/ros2-architecture',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub Discussions',
              href: 'https://github.com/your-username/Physical_AI_Humanoid_Robotics/discussions',
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
            {
              label: 'GitHub',
              href: 'https://github.com/your-username/Physical_AI_Humanoid_Robotics',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'json'],
    },

    // Table of contents settings
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 4,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
