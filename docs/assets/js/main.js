(function () {
  'use strict';
  // 行動版選單
  const toggle = document.querySelector('.nav-toggle');
  const links = document.querySelector('.nav-links');
  if (toggle && links) {
    toggle.addEventListener('click', () => {
      const open = links.classList.toggle('open');
      toggle.setAttribute('aria-expanded', String(open));
    });
    links.addEventListener('click', (e) => {
      if (e.target.tagName === 'A') { links.classList.remove('open'); toggle.setAttribute('aria-expanded', 'false'); }
    });
  }

  // 目前章節高亮
  const navAnchors = [...document.querySelectorAll('.nav-links a[href^="#"]')];
  const sections = navAnchors.map((a) => document.querySelector(a.getAttribute('href'))).filter(Boolean);
  if ('IntersectionObserver' in window) {
    const io = new IntersectionObserver((entries) => {
      entries.forEach((en) => {
        if (!en.isIntersecting) return;
        navAnchors.forEach((a) => a.setAttribute('aria-current', String(a.getAttribute('href') === '#' + en.target.id)));
      });
    }, { rootMargin: '-45% 0px -50% 0px' });
    sections.forEach((s) => io.observe(s));
  }

  // 影片為 HEVC (hvc1)；瀏覽器不支援時顯示下載提示
  document.querySelectorAll('video[data-hevc]').forEach((v) => {
    const note = v.parentElement.querySelector('.video-fallback');
    const support = v.canPlayType('video/mp4; codecs="hvc1"') || v.canPlayType('video/mp4; codecs="hvc1.1.6.L93.B0"');
    const show = () => note && note.classList.add('show');
    if (!support) show();
    v.addEventListener('error', show, true);
    const src = v.querySelector('source');
    if (src) src.addEventListener('error', show);
  });

  // KaTeX 數學排版（CDN 載入失敗時保留 TeX 原文）
  // defer 載入的 KaTeX 會在 DOMContentLoaded 之前執行完；不等 load，避免被影片下載拖延
  document.addEventListener('DOMContentLoaded', () => {
    if (typeof window.renderMathInElement !== 'function') {
      document.documentElement.classList.add('no-katex');
      return;
    }
    window.renderMathInElement(document.body, {
      delimiters: [
        { left: '$$', right: '$$', display: true },
        { left: '\\(', right: '\\)', display: false },
      ],
      throwOnError: false,
      ignoredClasses: ['packet', 'no-math'],
    });
  });
})();
