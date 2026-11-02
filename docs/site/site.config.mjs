// The Molly documentation site: which Markdown of this repository becomes which page.
// Every page stays a file of the repository (README, docs/, package READMEs); scripts/sync.mjs
// copies them.
const pkg = (name) => ({ src: `src/${name}/README.md`, dest: `packages/${name}`, text: name })

export default {
  title: 'Molly',
  description:
    'Tour guide robot on the Deep Robotics Lite3 and the Dharma stack: tours between points of ' +
    'interest with narration, a greeter and conversation, a handheld and a web dashboard.',
  repo: 'DarmawanRobotics/Molly',
  branch: 'main',
  logo: 'logo.svg',
  brand: { light: '#c94f33', light2: '#e0603f', dark: '#ff9372', dark2: '#ffb39a', soft: 'rgba(224,96,63,.14)' },
  copyright: 'Apache-2.0 · DarmawanRobotics',
  sections: [
    {
      text: 'Guide',
      items: [
        { src: 'README.md', dest: 'guide/overview', text: 'Overview' },
        { src: 'docs/time.md', dest: 'guide/time', text: 'Time synchronisation' },
        { src: 'docker/README.md', dest: 'guide/dev-container', text: 'Dev container' },
      ],
    },
    {
      text: 'Reference',
      items: [
        { src: 'docs/interfaces.md', dest: 'reference/interfaces', text: 'Interfaces' },
        { src: 'docs/web.md', dest: 'reference/dashboard', text: 'Dashboard contract' },
        { src: 'src/molly_bridge/PROTOCOL.md', dest: 'reference/handheld-protocol', text: 'Handheld protocol' },
        { src: 'src/xtras/README.md', dest: 'reference/robot-packages', text: 'Robot packages' },
        { src: 'docs/README.md', dest: 'reference/documents', text: 'Documents' },
      ],
    },
    {
      text: 'Packages',
      items: [
        pkg('molly_bringup'),
        pkg('molly_guide'),
        pkg('molly_interaction'),
        pkg('molly_bridge'),
        pkg('molly_interfaces'),
      ],
    },
    {
      text: 'Project',
      items: [
        { src: 'CONTRIBUTING.md', dest: 'project/contributing', text: 'Contributing' },
        { src: 'CHANGELOG.md', dest: 'project/changelog', text: 'Changelog' },
      ],
    },
  ],
  files: [],
}
