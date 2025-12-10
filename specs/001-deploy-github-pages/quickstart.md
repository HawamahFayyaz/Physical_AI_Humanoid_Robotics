# Quickstart: GitHub Pages Deployment

## Overview
This guide provides quick setup instructions for deploying the Physical AI & Humanoid Robotics book to GitHub Pages with automated deployment and performance optimization.

## Prerequisites
- Node.js 18+ installed
- Git installed
- GitHub account with repository access
- Basic understanding of GitHub Actions

## Local Development Setup

1. **Clone the repository**:
   ```bash
   git clone https://github.com/[your-username]/Physical_AI_Humanoid_Robotics.git
   cd Physical_AI_Humanoid_Robotics
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Start local development server**:
   ```bash
   npm run start
   ```
   The site will be available at http://localhost:3000

## GitHub Pages Deployment Configuration

1. **Enable GitHub Pages** in your repository settings:
   - Go to Settings → Pages
   - Select "GitHub Actions" as the source

2. **The deployment workflow** (`.github/workflows/deploy.yml`) will automatically:
   - Build the Docusaurus site on pushes to main
   - Deploy to GitHub Pages
   - Optimize for performance

## Performance Optimization

The deployment process includes:
- Image optimization to WebP format
- JavaScript bundle minimization
- Critical CSS inlining
- Proper meta tags and structured data
- ARIA labels for accessibility

## Lighthouse Scoring

To verify Lighthouse 100/100 scores locally:
```bash
npm run build
npx serve -s build
# Then run Lighthouse audit in Chrome DevTools
```

## Customization

1. **Modify docusaurus.config.js** to update site metadata
2. **Update README.md** with project-specific information
3. **Customize styling** in the src/css directory

## Troubleshooting

- **Build failures**: Check that all markdown files are properly formatted
- **Performance issues**: Run `npm run build` locally to identify issues
- **Deployment errors**: Check GitHub Actions logs in the Actions tab

## Next Steps

After successful deployment:
1. Verify the site is live at `https://[your-username].github.io/Physical_AI_Humanoid_Robotics/`
2. Run Lighthouse audit to confirm 100/100 scores
3. Test all functionality including navigation and dark/light mode
4. Update the README with your specific deployment URL