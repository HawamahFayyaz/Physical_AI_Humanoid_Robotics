name: Docusaurus Config Patterns
description: Production Docusaurus configuration
Docusaurus Config
docusaurus.config.js
JavaScript

const config = {
  title: 'Physical AI & Humanoid Robotics',
  url: 'https://username.github.io',
  baseUrl: '/Physical_AI_Humanoid_Robotics/',
  onBrokenLinks: 'throw',
  i18n: { defaultLocale: 'en', locales: ['en', 'ur'] },
  presets: [['classic', {
    docs: { sidebarPath: './sidebars.js', routeBasePath: '/' },
    theme: { customCss: './src/css/custom.css' }
  }]],
  themes: ['@docusaurus/theme-mermaid'],
  markdown: { mermaid: true }
};
module.exports = config;
sidebars.js
JavaScript

module.exports = {
  bookSidebar: [
    { type: 'category', label: '🚀 Getting Started', items: ['intro/welcome'] },
    { type: 'category', label: '🧠 Foundations', items: ['foundations/overview'] }
  ]
};
