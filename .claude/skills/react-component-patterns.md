name: React Component Patterns
description: Reusable React components for Docusaurus
React Components
PersonalizeButton
React

export function PersonalizeButton({ chapterId, content }) {
  const [loading, setLoading] = useState(false);
  const handleClick = async () => {
    setLoading(true);
    const res = await fetch('/api/personalize', {
      method: 'POST',
      body: JSON.stringify({ chapterId, content })
    });
    setLoading(false);
  };
  return <button onClick={handleClick}>{loading ? '✨...' : '✨ Personalize'}</button>;
}
UrduButton
React

export function UrduButton({ content }) {
  const [urdu, setUrdu] = useState(null);
  const translate = async () => {
    const res = await fetch('/api/translate', {
      method: 'POST',
      body: JSON.stringify({ content, lang: 'ur' })
    });
    setUrdu((await res.json()).translation);
  };
  return (
    <>
      <button onClick={translate}>🇵🇰 اردو</button>
      {urdu && <div dir="rtl">{urdu}</div>}
    </>
  );
}
