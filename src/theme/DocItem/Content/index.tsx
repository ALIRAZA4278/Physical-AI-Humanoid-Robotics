/**
 * Custom DocItem/Content wrapper for Docusaurus.
 * Injects ChapterControls at the start of each documentation page.
 */

import React from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import ChapterControls from '../../../components/ChapterControls';
import { useDoc } from '@docusaurus/plugin-content-docs/client';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  const { metadata } = useDoc();
  const title = metadata?.title || 'Chapter';

  // Get the content element to extract text for personalization
  const contentRef = React.useRef<HTMLDivElement>(null);
  const [chapterContent, setChapterContent] = React.useState<string>('');

  React.useEffect(() => {
    // Extract text content after render for personalization/translation
    if (contentRef.current) {
      const text = contentRef.current.textContent || '';
      // Limit to first 5000 chars for API calls
      setChapterContent(text.slice(0, 5000));
    }
  }, []);

  return (
    <div ref={contentRef}>
      <ChapterControls chapterTitle={title} chapterContent={chapterContent} />
      <Content {...props} />
    </div>
  );
}
