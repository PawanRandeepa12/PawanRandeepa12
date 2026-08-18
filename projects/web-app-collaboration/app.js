// Web App Collaboration - Starter JavaScript

console.log('%c[WebAppCollab] Starter script loaded.', 'color:#64748b');

document.addEventListener('DOMContentLoaded', () => {
    // CTA Button in navbar
    const ctaBtn = document.getElementById('cta-btn');
    if (ctaBtn) {
        ctaBtn.addEventListener('click', () => {
            const features = document.getElementById('features');
            if (features) {
                features.scrollIntoView({ behavior: 'smooth', block: 'start' });
            }
        });
    }

    // Live Demo button
    const demoBtn = document.getElementById('demo-btn');
    if (demoBtn) {
        demoBtn.addEventListener('click', () => {
            const originalText = demoBtn.textContent;
            demoBtn.textContent = 'Launching demo...';
            demoBtn.disabled = true;

            // Simulate a demo action
            setTimeout(() => {
                showToast('Demo mode activated! This is a placeholder for the real web app.');
                demoBtn.textContent = originalText;
                demoBtn.disabled = false;

                // Optional: highlight a feature card
                const firstCard = document.querySelector('.card');
                if (firstCard) {
                    firstCard.style.transition = 'box-shadow 0.3s ease';
                    firstCard.style.boxShadow = '0 0 0 3px #6366f1';
                    
                    setTimeout(() => {
                        firstCard.style.boxShadow = '';
                    }, 1400);
                }
            }, 650);
        });
    }

    // Join project button
    const joinBtn = document.getElementById('join-btn');
    if (joinBtn) {
        joinBtn.addEventListener('click', () => {
            const contactSection = document.getElementById('contact');
            if (contactSection) {
                contactSection.scrollIntoView({ behavior: 'smooth' });
            }
            
            setTimeout(() => {
                showToast('Thanks! Check the README for instructions to push dev/cw in a new session.');
            }, 800);
        });
    }

    // Easter egg: click logo 5 times
    let logoClicks = 0;
    const logo = document.querySelector('.logo');
    if (logo) {
        logo.style.cursor = 'pointer';
        logo.addEventListener('click', () => {
            logoClicks++;
            if (logoClicks === 5) {
                logoClicks = 0;
                logo.style.transition = 'transform 0.6s cubic-bezier(0.68, -0.55, 0.27, 1.55)';
                logo.style.transform = 'rotate(360deg) scale(1.4)';
                
                setTimeout(() => {
                    logo.style.transform = '';
                    showToast('🎉 You found the collaboration easter egg!');
                }, 700);
            }
        });
    }

    // Keyboard shortcut hint
    document.addEventListener('keydown', (e) => {
        if (e.key === '?' && document.activeElement.tagName === 'BODY') {
            e.preventDefault();
            const features = document.getElementById('features');
            if (features) features.scrollIntoView({ behavior: 'smooth' });
        }
    });

    console.log('%c[WebAppCollab] All interactive elements initialized.', 'color:#64748b');
});

// Simple toast notification
function showToast(message) {
    const toast = document.createElement('div');
    toast.style.cssText = `
        position: fixed;
        bottom: 24px;
        left: 50%;
        transform: translateX(-50%);
        background: #1e293b;
        color: #f1f5f9;
        padding: 14px 22px;
        border-radius: 9999px;
        box-shadow: 0 10px 15px -3px rgb(0 0 0 / 0.2);
        border: 1px solid #334155;
        font-size: 0.95rem;
        z-index: 9999;
        white-space: nowrap;
        display: flex;
        align-items: center;
        gap: 8px;
    `;
    toast.innerHTML = `
        <span>${message}</span>
    `;
    
    document.body.appendChild(toast);

    setTimeout(() => {
        toast.style.transition = 'opacity 0.3s ease, transform 0.3s ease';
        toast.style.opacity = '0';
        toast.style.transform = 'translateX(-50%) translateY(10px)';
        
        setTimeout(() => {
            toast.remove();
        }, 200);
    }, 2800);
}

// Export for future module usage
window.WebAppCollab = {
    showToast,
    version: '0.1.0-collaboration'
};
