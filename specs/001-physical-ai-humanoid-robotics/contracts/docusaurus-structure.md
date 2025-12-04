# Docusaurus Structure Contract

**Purpose**: Define the file hierarchy and configuration for the Physical AI & Humanoid Robotics Docusaurus book.

---

## Project Structure

```text
my-research-paper/
├── docs/                              # Content root
│   ├── intro.md                       # Book introduction (sidebar_position: 1)
│   │
│   ├── module-1-ros2/                 # Module 1: ROS 2 Fundamentals
│   │   ├── _category_.json
│   │   ├── 01-architecture.md         # 1.1 ROS 2 Architecture
│   │   ├── 02-first-node.md           # 1.2 Building Your First Node
│   │   ├── 03-urdf.md                 # 1.3 URDF Robot Description
│   │   └── 04-environment.md          # 1.4 Development Environment
│   │
│   ├── module-2-simulation/           # Module 2: Digital Twin
│   │   ├── _category_.json
│   │   ├── 01-gazebo-setup.md         # 2.1 Gazebo + ROS 2
│   │   ├── 02-sdf-modeling.md         # 2.2 SDF Robot Modeling
│   │   ├── 03-sensors.md              # 2.3 Sensor Simulation
│   │   └── 04-unity-viz.md            # 2.4 Unity Visualization
│   │
│   ├── module-3-isaac/                # Module 3: NVIDIA Isaac
│   │   ├── _category_.json
│   │   ├── 01-isaac-setup.md          # 3.1 Isaac Sim Setup
│   │   ├── 02-perception.md           # 3.2 Perception Pipelines
│   │   └── 03-nav2.md                 # 3.3 Nav2 Navigation
│   │
│   ├── module-4-vla/                  # Module 4: VLA Capstone
│   │   ├── _category_.json
│   │   ├── 01-whisper.md              # 4.1 Voice-to-Text
│   │   ├── 02-planning.md             # 4.2 Cognitive Planning
│   │   └── 03-capstone.md             # 4.3 Full VLA Project
│   │
│   ├── glossary.md                    # Technical terms A-Z
│   │
│   └── appendix/                      # Reference material
│       ├── _category_.json
│       ├── hardware-requirements.md
│       ├── version-migration.md
│       └── troubleshooting.md
│
├── src/                               # React customization
│   ├── components/                    # Custom components
│   │   └── HardwareCheck.tsx          # GPU requirement banner
│   ├── css/
│   │   └── custom.css                 # Brand styling
│   └── pages/
│       └── index.tsx                  # Landing page
│
├── static/                            # Static assets
│   ├── img/
│   │   ├── architecture/              # System diagrams
│   │   ├── screenshots/               # Tool screenshots
│   │   └── logo.svg                   # Book logo
│   └── code/
│       ├── module-1/                  # Downloadable examples
│       ├── module-2/
│       ├── module-3/
│       └── module-4/
│
├── docusaurus.config.js               # Site configuration
├── sidebars.js                        # Navigation structure
├── package.json                       # Dependencies
├── tsconfig.json                      # TypeScript config
└── babel.config.js                    # Babel config
```

---

## Configuration Files

### docusaurus.config.js

```javascript
// @ts-check
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From ROS 2 to Voice-Controlled Autonomous Humanoids',
  favicon: 'img/favicon.ico',
  url: 'https://your-github-username.github.io',
  baseUrl: '/my-research-paper/',

  organizationName: 'your-github-username',
  projectName: 'my-research-paper',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/your-github-username/my-research-paper/edit/main/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/your-github-username/my-research-paper',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      copyright: `Copyright © ${new Date().getFullYear()}. Built with Docusaurus.`,
    },
    prism: {
      theme: require('prism-react-renderer').themes.github,
      darkTheme: require('prism-react-renderer').themes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'json', 'xml'],
    },
  },
};

module.exports = config;
```

---

### sidebars.js

```javascript
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1-ros2/01-architecture',
        'module-1-ros2/02-first-node',
        'module-1-ros2/03-urdf',
        'module-1-ros2/04-environment',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      items: [
        'module-2-simulation/01-gazebo-setup',
        'module-2-simulation/02-sdf-modeling',
        'module-2-simulation/03-sensors',
        'module-2-simulation/04-unity-viz',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac AI',
      items: [
        'module-3-isaac/01-isaac-setup',
        'module-3-isaac/02-perception',
        'module-3-isaac/03-nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Capstone',
      items: [
        'module-4-vla/01-whisper',
        'module-4-vla/02-planning',
        'module-4-vla/03-capstone',
      ],
    },
    'glossary',
    {
      type: 'category',
      label: 'Appendix',
      items: [
        'appendix/hardware-requirements',
        'appendix/version-migration',
        'appendix/troubleshooting',
      ],
    },
  ],
};

module.exports = sidebars;
```

---

### _category_.json Template

Each module folder MUST contain:

```json
{
  "label": "Module {N}: {Title}",
  "position": {N},
  "link": {
    "type": "generated-index",
    "description": "{Module description}"
  },
  "customProps": {
    "hardware_required": false,
    "estimated_hours": 10
  }
}
```

**Module 3 Example** (hardware required):

```json
{
  "label": "Module 3: NVIDIA Isaac AI",
  "position": 3,
  "link": {
    "type": "generated-index",
    "description": "Learn AI perception and navigation with NVIDIA Isaac Sim"
  },
  "customProps": {
    "hardware_required": true,
    "gpu_minimum": "RTX 3070",
    "estimated_hours": 12
  }
}
```

---

## Package Dependencies

### package.json

```json
{
  "name": "physical-ai-humanoid-robotics",
  "version": "1.0.0",
  "private": true,
  "scripts": {
    "docusaurus": "docusaurus",
    "start": "docusaurus start",
    "build": "docusaurus build",
    "serve": "docusaurus serve",
    "clear": "docusaurus clear",
    "deploy": "docusaurus deploy"
  },
  "dependencies": {
    "@docusaurus/core": "^3.6.0",
    "@docusaurus/preset-classic": "^3.6.0",
    "@mdx-js/react": "^3.0.0",
    "clsx": "^2.0.0",
    "prism-react-renderer": "^2.3.0",
    "react": "^18.2.0",
    "react-dom": "^18.2.0"
  },
  "devDependencies": {
    "@docusaurus/module-type-aliases": "^3.6.0",
    "@docusaurus/tsconfig": "^3.6.0",
    "typescript": "~5.2.2"
  },
  "engines": {
    "node": ">=18.0"
  }
}
```

---

## Validation Commands

```bash
# Install dependencies
npm install

# Start development server
npm start

# Build for production (validates all content)
npm run build

# Serve production build locally
npm run serve

# Deploy to GitHub Pages
npm run deploy
```

---

## Success Criteria

Per Constitution Success Criteria:

- [ ] `npm run build` passes without errors (SC-008)
- [ ] All sidebar links resolve to valid chapters
- [ ] All images load correctly
- [ ] Prism syntax highlighting works for all languages
- [ ] Mobile-responsive layout verified
- [ ] GitHub Pages deployment completes (SC-008)
