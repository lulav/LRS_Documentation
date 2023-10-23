module.exports = {
  citrosSidebar: [
    {
      type: "category",
      label: "CITRSO Web",
      link: {
        type: "doc",
        id: "index",
      },
      items: [
        {
          type: "doc",
          id: "quickstart/quick_tbd",
          label: "Quick Start",
        },
      ],
    
    },

     //Authentication
    {
      type: 'category',
      label: 'Account',
      items: [
        {
          type: 'doc',
          label: 'Overview',
          id: 'authentication/account/account_overview',
        },
        {
          type: 'doc',
          label: 'Account Registration',
          id: 'authentication/account/account_login',
        },
        {
          type: 'doc',
          label: 'Change Password',
          id: 'authentication/account/account_password',
        },
        {
          type: 'category',
          label: 'Profile Settings',
          items: [
            {
              type: 'doc',
              label: 'Notification Settings',
              id: 'authentication/account/profile/notification_settings',
            },
            {
              type: 'doc',
              label: 'SSH Keys',
              id: 'authentication/account/profile/ssh',
            },
            {
              type: 'doc',
              label: 'Quota',
              id: 'authentication/account/profile/quota',
            },
              ]
            },
          ]
        },
    {
      type: 'category',
      label: 'SSH Keys',
      items: [
        {
          type: 'doc',
          label: 'Overview',
          id: 'authentication/ssh/ssh_overview',
        },
        {
          type: 'category',
          label: 'Connect with SSH',
          items: [
            {
              type: 'doc',
              label: 'SSH Agent Forwarding',
              id: 'authentication/ssh/ssh_agent_forwarding',
            },
            {
              type: 'doc',
              label: 'Managing deploy keys',
              id: 'authentication/ssh/ssh_mng_deploy_keys',
            },
            {
              type: 'doc',
              label: 'Checking for existing SSH keys',
              id: 'authentication/ssh/ssh_chk_existing_key',
            },
            {
              type: 'doc',
              label: 'Generate new SSH Key',
              id: 'authentication/ssh/ssh_generate_key',
            },
            {
              type: 'doc',
              label: 'Adding a new SSH key',
              id: 'authentication/ssh/ssh_add_new',
            },
            {
              type: 'doc',
              label: 'Test your SSH connection',
              id: 'authentication/ssh/ssh_test_connection',
            },
            {
              type: 'doc',
              label: 'SSH key passphrases',
              id: 'authentication/ssh/ssh_passphrases',
            }
          ]
        },
        {
          type: 'category',
          label: 'Troubleshooting',
          items: [
            {
              type: 'doc',
              label: 'SSH key passphrases',
              id: 'authentication/troubleshooting/ts_passphrases',
            },
            {
              type: 'doc',
              label: 'Deleted or Missing SSH Keys',
              id: 'authentication/troubleshooting/ts_missing_key',
            },
            {
              type: 'doc',
              label: 'Errors',
              id: 'authentication/troubleshooting/ts_error',
            },
            {
              type: 'doc',
              label: 'Permission Denied',
              id: 'authentication/troubleshooting/ts_permissions',
            }
          ]
        },
      ]
    },
  
    
    //Repositories
    {
      type: 'category',
      label: 'Repositories',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          label: 'Overview',
          id: 'repos/repos_overview',
        },
        {
          type: 'doc',
          label: 'Repositories Screen',
          id: 'repos/repos_screen',
        },
        {
          type: 'category',
          label: 'Repositry Tabs',
          items: [
            {
              type: 'doc',
              label: 'Overview',
              id: 'repos/repository/repo_sc_overview',
                        },
            {
              type: 'doc',
              label: 'Code',
              id: 'repos/repository/repo_sc_code',
            },
            {
              type: 'doc',
              label: 'Runs',
              id: 'repos/repository/repo_sc_runs',
            },
            {
              type: 'doc',
              label: 'Images',
              id: 'repos/repository/repo_sc_images',
            },
            {
              type: 'doc',
              label: 'Data',
              id: 'repos/repository/repo_sc_data',
            },
          ]
        },
{
          type: 'doc',
          label: 'Add Repository',
          id: 'repos/repos_add',
        },

      ],
    },

    //Simulations
    {
      type: 'category',
      label: 'Simulations',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          label: 'Overview',
          id: 'simulations/sim_overview',
        },
        {
          type: 'category',
          label: 'Batch Simulations',
          items: [
            // {
            //   type: 'doc',
            //   label: 'Breadcrumbs',
            //   id: 'simulations_batch/sim_batch_breadcrumbs',
            // },
            {
              type: 'doc',
              label: 'Run New Batch Simulations',
              id: 'simulations/simulations_batch/sim_batch_run',
            },
            {
              type: 'doc',
              label: 'Batch Simulations Screen',
              id: 'simulations/simulations_batch/sim_batch_screen',
            },
          ]
        },
        {
          type: 'category',
          label: 'Simulation Screen',
          items: [

            {
              type: 'doc',
              label: 'Possible Runs',
              id: 'simulations/simulation_screen/sim_possible_runs',
            },
            {
              type: 'doc',
              label: 'Logs',
              id: 'simulations/simulation_screen/sim_logs',
            },
          ]
        },
      ],
    },

    //Billing
    // {
    //   type: 'category',
    //   label: 'Billing',
    //   collapsible: true,
    //   collapsed: true,
    //   items: [
    //     {
    //       type: 'doc',
    //       label: 'Payment Method',
    //       id: 'billing/billing_pm',
    //     },
    //     {
    //       type: 'doc',
    //       label: 'Billing Address',
    //       id: 'billing/billing_address',
    //     },
    //     {
    //       type: 'doc',
    //       label: 'Invoices',
    //       id: 'billing/billing_invoices',
    //     },
    //     {
    //       type: 'doc',
    //       label: 'Plans',
    //       id: 'billing/billing_plans',
    //     },
    //   ],
    // },
    
    //Organization
    // {
    //   type: 'category',
    //   label: 'Organization',
    //   collapsible: true,
    //   collapsed: true,
    //   items: [
    //     {
    //       type: 'doc',
    //       label: 'TBD',
    //       id: 'organization/organization_tbd',
    //     },
    //   ],
    // },
  ],
};