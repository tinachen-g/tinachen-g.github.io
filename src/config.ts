export const siteConfig: SiteConfig = {
    title: "Tina Cheng",
    language: "en",
    description: "𐔌՞. .՞𐦯",
    keywords: "blog",
    author: "Tina Cheng",
    avatar: "/avatar.png",
    favicon: "/favicon.png",
    site: "https://tinachen-g.github.io",

    page_size: 10,
}

export const navBarConfig: NavBarConfig = {
    links: [
        {
            name: 'About',
            url: '/about'
        },
        {
            name: 'Projects',
            url: '/projects'
        },
        // {
        //     name: 'Links',
        //     url: '/links'
        // },

    ]
}

export const socialLinks: SocialLink[] = [
    // https://icon-sets.iconify.design/material-symbols/
    {
        label: 'GitHub',
        icon: 'mdi-github',
        url: 'https://github.com/tinachen-g'
    },
    {
        label: 'Linkedin',
        icon: 'mdi-linkedin',
        url: 'https://www.linkedin.com/in/tina-chen-g/'
    }
]

interface SiteConfig {
    title: string
    language: string
    description: string
    keywords: string
    author: string
    avatar: string
    favicon: string
    site: string

    page_size: number
    twikoo_uri?: string     // https://twikoo.js.org/
}

interface NavBarConfig {
    links: {
        name: string
        url: string
        target?: string
    }[]
}

interface SocialLink {
    label: string
    icon: string
    url: string
}
