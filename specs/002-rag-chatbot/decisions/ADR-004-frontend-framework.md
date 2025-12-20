# ADR-004: Frontend Chat Component Framework

**Status**: Accepted
**Date**: 2025-12-17
**Decision Makers**: Development Team
**Feature**: 002-rag-chatbot

## Context

The RAG chatbot needs a frontend component integrated into the Docusaurus documentation site. The component must:
- Display chat messages with markdown formatting
- Show source citations as clickable links
- Support text selection detection on chapter pages
- Match the Docusaurus theme (light/dark mode)
- Be responsive on mobile devices (320px+)
- Support keyboard accessibility

## Decision

**Build a custom React/TypeScript component** specifically designed for the Docusaurus integration.

## Options Considered

### Option 1: OpenAI ChatKit SDK
- **Pros**: Pre-built UI, streaming support, quick setup
- **Cons**: Limited customization, may not match Docusaurus theme, large bundle
- **Effort**: Low

### Option 2: Vercel AI SDK UI
- **Pros**: Streaming hooks, React-native, well-maintained
- **Cons**: Requires custom UI, focused on Next.js patterns
- **Effort**: Medium

### Option 3: Custom React Component (CHOSEN)
- **Pros**: Full control, perfect theme match, no bundle bloat
- **Cons**: Build from scratch, more development time
- **Effort**: High

### Option 4: react-chat-widget
- **Pros**: Ready-made floating widget
- **Cons**: Outdated (last update 2021), hard to customize, doesn't match modern React patterns
- **Effort**: Low

### Option 5: @chatscope/chat-ui-kit-react
- **Pros**: Professional components, comprehensive
- **Cons**: Heavy bundle (~50KB), learning curve, over-engineered for this use case
- **Effort**: Medium

## Rationale

1. **Theme Integration**: Docusaurus uses CSS variables for theming. A custom component can directly use these variables (`--ifm-color-primary`, `--ifm-background-color`, etc.) for perfect light/dark mode support.

2. **Text Selection Feature**: The text selection feature requires deep integration with page content. Pre-built chat libraries don't support this, requiring custom code anyway.

3. **Bundle Size**: A custom component adds ~10KB to the bundle vs 50-100KB for full chat libraries. This matters for Lighthouse scores (Constitution Principle V).

4. **Citation Links**: No chat library has built-in support for clickable citations that navigate within Docusaurus. Custom rendering is required.

5. **Maintenance**: No external dependency updates to track. The component is tailored to exactly this use case.

6. **Reusability**: Per Constitution Principle II, the component can be extracted and reused in future projects with minimal modification.

## Implementation

### Component Architecture

```
ChatBot/
├── index.tsx              # Main export, context provider
├── ChatWidget.tsx         # Floating FAB + collapsible panel
├── ChatPanel.tsx          # Header + message list + input
├── ChatMessage.tsx        # Single message with citations
├── TextSelection.tsx      # Selection detector + floating button
├── LoadingIndicator.tsx   # Typing animation
├── ErrorBoundary.tsx      # Error state handling
├── hooks/
│   ├── useChat.ts         # Chat state, API calls, session
│   ├── useTextSelection.ts # Selection API wrapper
│   └── useLocalStorage.ts  # Persistent session storage
├── types.ts               # TypeScript interfaces
├── api.ts                 # Backend client
├── constants.ts           # Config values
└── styles.module.css      # CSS Modules
```

### Key Components

#### ChatWidget.tsx
```tsx
export function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const { messages, context, setContext } = useChat();

  return (
    <>
      <TextSelection onSelect={(text) => {
        setContext({ selectedText: text });
        setIsOpen(true);
      }} />

      <button
        className={styles.fab}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isOpen ? <CloseIcon /> : <ChatIcon />}
      </button>

      {isOpen && <ChatPanel />}
    </>
  );
}
```

#### Theme Integration
```css
/* styles.module.css */
.panel {
  background: var(--ifm-background-color);
  color: var(--ifm-font-color-base);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: var(--ifm-border-radius);
}

.userMessage {
  background: var(--ifm-color-primary);
  color: var(--ifm-color-primary-contrast-foreground);
}

.assistantMessage {
  background: var(--ifm-color-emphasis-100);
}
```

#### Mobile Responsiveness
```css
@media (max-width: 768px) {
  .panel {
    position: fixed;
    inset: 0;
    width: 100%;
    height: 100%;
    border-radius: 0;
    z-index: 1000;
  }

  .fab {
    bottom: 16px;
    right: 16px;
    width: 56px;
    height: 56px;
  }
}
```

## Consequences

### Positive
- Perfect visual consistency with Docusaurus theme
- Minimal bundle impact (~10KB)
- Full control over behavior and animations
- Clean integration with text selection feature
- No third-party dependency risks

### Negative
- Higher initial development effort
- Must implement markdown rendering manually
- Responsible for accessibility compliance
- No pre-built streaming animation

### Mitigations
- Use `react-markdown` for message rendering (small, well-maintained)
- Follow WCAG 2.1 guidelines from the start
- Use CSS animations for loading states
- Test with screen readers during development

## Accessibility Requirements

| Requirement | Implementation |
|-------------|----------------|
| Keyboard navigation | Tab through messages, Enter to send |
| Screen reader | ARIA labels, role="dialog", live regions |
| Focus management | Focus trap in panel, return focus on close |
| Color contrast | Inherit from Docusaurus (already compliant) |
| Touch targets | Minimum 44x44px for mobile buttons |

## Integration with Docusaurus

```tsx
// src/theme/Root.tsx
import { ChatBot } from '@site/src/components/ChatBot';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatBot
        apiUrl={process.env.CHATBOT_API_URL || 'http://localhost:8000'}
      />
    </>
  );
}
```

## References

- [Docusaurus Swizzling](https://docusaurus.io/docs/swizzling)
- [React ARIA](https://react-spectrum.adobe.com/react-aria/)
- [CSS Variables in Docusaurus](https://docusaurus.io/docs/styling-layout#styling-your-site-with-infima)
- [react-markdown](https://github.com/remarkjs/react-markdown)
