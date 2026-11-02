---
layout: home

hero:
  name: Molly
  text: A tour guide that walks with you
  tagline: >-
    Molly walks visitors from one point of interest to the next on a Deep Robotics Lite3, tells
    them about each one, waits when they fall behind and greets people who come up to it. Driven
    from a handheld and a web dashboard, built on the Dharma stack.
  image:
    src: /logo.svg
    alt: Molly
  actions:
    - theme: brand
      text: Quick start
      link: /guide/overview#quick-start
    - theme: alt
      text: Install on the robot
      link: /guide/overview#install
    - theme: alt
      text: Interfaces
      link: /reference/interfaces

features:
  - icon: 🗺️
    title: Tours between points of interest
    details: >-
      A map recorded with the AprilTags that anchor it, points of interest on it, and tours that
      walk them in order with the path finder.
    link: /packages/molly_guide
    linkText: molly_guide
  - icon: 🗣️
    title: Narration and conversation
    details: >-
      Each stop is told in Indonesian or English with Piper, and visitors can ask questions a
      local language model answers.
    link: /packages/molly_interaction
    linkText: molly_interaction
  - icon: 👋
    title: Greeter and followers
    details: >-
      The 360° LiDAR sees who walks behind and who comes up: Molly waits for the group and greets
      newcomers with a gesture.
    link: /packages/molly_interaction
    linkText: Visitor tracker
  - icon: 🎮
    title: Handheld and dashboard
    details: >-
      A handheld drives and stops the robot; the web dashboard runs tours, shows the map, the
      camera and the robot's health.
    link: /reference/dashboard
    linkText: Dashboard contract
  - icon: 🛡️
    title: Safe on Dharma
    details: >-
      Odometry, localization, the path follower, the protective field and the e-stop come from the
      shared Dharma stack.
    link: https://darmawanrobotics.github.io/Dharma/
    linkText: Dharma
  - icon: 🕒
    title: One clock
    details: >-
      The Mid-360, the camera and the computers on one clock: chrony and PTP set up by one command.
    link: /guide/time
    linkText: Time synchronisation
---
