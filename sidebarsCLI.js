module.exports = {
  cliSidebar: [
    {
      type: "category",
      label: "CITROS CLI",
      link: {
        type: "doc",
        id: "index",
      },
      items: [
        {
          type: "doc",
          id: "cli_overview",
          label: "Overview",
        },
      ],
    
    },

     //Commands
    {
      type: 'category',
      label: 'CLI Commands',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          // label: 'Quick Start',
          id: 'commands/quick_start',
        },
        {
          type: 'doc',
          // label: 'init',
          id: 'commands/init',
        },
        {
          type: 'doc',
          // label: 'setup-ssh',
          id: 'commands/setup-ssh',
        },
        {
          type: 'doc',
          // label: 'status',
          id: 'commands/status',
        },
        {
          type: 'doc',
          label: 'add-remote',
          id: 'commands/add-remote',
        },
        {
          type: 'doc',
          // label: 'commit',
          id: 'commands/commit',
        },
        {
          type: 'doc',
          // label: 'pull',
          id: 'commands/pull',
        },
        {
          type: 'doc',
          // label: 'push',
          id: 'commands/push',
        },
        {
          type: 'doc',
          // label: 'diff',
          id: 'commands/diff',
        },
        {
          type: 'doc',
          // label: 'checkout',
          id: 'commands/checkout',
        },
        {
          type: 'doc',
          // label: 'merge',
          id: 'commands/merge',
        },
        {
          type: 'doc',
          // label: 'discard',
          id: 'commands/discard',
        },
        {
          type: 'doc',
          // label: 'login',
          id: 'commands/login',
        },
        {
          type: 'doc',
          // label: 'logout',
          id: 'commands/logout',
        },
        {
          type: 'doc',
          // label: 'list',
          id: 'commands/list',
        },
        {
          type: 'doc',
          // label: 'run',
          id: 'commands/run',
        },
        {
          type: 'doc',
          // label: 'docker-build',
          id: 'commands/docker-build',
        },
        {
          type: 'doc',
          // label: 'docker-build-push',
          id: 'commands/docker-build-push',
        },
      ],
    },

    //structure
    {
      type: 'category',
      label: 'Citros Repository Directory and File Structure',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          //label: 'notebooks',
          id: 'structure/notebooks',
        },
        {
          type: 'doc',
          // label: 'parameter setups',
          id: 'structure/paramater_setups',
        },
        {
          type: 'doc',
          // label: 'reports',
          id: 'structure/reports',
        },
        {
          type: 'doc',
          // label: 'runs',
          id: 'structure/runs',
        },
        {
          type: 'doc',
          // label: 'simulations',
          id: 'structure/simulations',
        },
        {
          type: 'doc',
          // label: 'workflows',
          id: 'structure/workflows',
        },
        {
          type: 'doc',
          // label: 'project.json',
          id: 'structure/project_json',
        },
        {
          type: 'doc',
          // label: 'settings.json',
          id: 'structure/settings_json',
        },
      ],
    },

     //repo config
     {
      type: 'category',
      label: 'Citros Repository Configuration',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          label: 'Adding functions to parameter setup',
          id: 'configuration/param_functions',
        },
      ],
    },
    {
      type: "doc",
      label: "User Templates",
      id: "user_templates",
    },
  
  ],
};


// 
