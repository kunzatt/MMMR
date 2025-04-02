package com.ssafy.mmmr.news.service;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.NewsException;
import com.ssafy.mmmr.news.dto.NewsResponseDto;
import com.ssafy.mmmr.news.entity.NewsEntity;
import com.ssafy.mmmr.news.repository.NewsRepository;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.jsoup.Jsoup;
import org.jsoup.nodes.Document;
import org.jsoup.nodes.Element;
import org.jsoup.select.Elements;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.stream.Collectors;

@Slf4j
@Service
@RequiredArgsConstructor
public class NewsService {

    private final NewsRepository newsRepository;

    public List<NewsResponseDto> getNewsList() {
            List<NewsEntity> newsEntities = newsRepository.findAll();
            if(newsEntities.isEmpty()) {
                throw new NewsException(ErrorCode.NEWS_FETCH_FAILED);
            }
            List<NewsResponseDto> list = newsEntities.stream()
                    .map(newsEntity -> new NewsResponseDto(
                            newsEntity.getId(),
                            newsEntity.getTitle(),
                            newsEntity.getContent()
                    ))
                    .collect(Collectors.toList());
            return list;
    }

    public void newsCrawler() {
        try{
            String pageUrl = "https://media.naver.com/press/001";

            Document document = Jsoup.connect(pageUrl).timeout(5000).get();
            // 메인 기사 : 총 5개
            Elements elements = document.select(".press_main_news_inner");
            Elements mainArticles = elements.select(".press_news_item");

            for(int i=0; i<5; i++){
                Element article = mainArticles.get(i);
                //개별 기사 제목
                String title = article.select("strong").text().trim();

                // 기사 url
                String articleUrl = article.selectFirst(".press_news_link").attr("href");

                // 기사 내용
                Document contentDocument = Jsoup.connect(articleUrl).timeout(5000).get();
                String content = contentDocument.select(".story-news.article").text();
                //DB에 저장하는 로직
                NewsEntity newsEntity = NewsEntity.builder()
                        .id((long)mainArticles.indexOf(article)+1)
                        .title(title)
                        .content(content)
                        .build();
                newsRepository.save(newsEntity);
                log.info("뉴스 크롤링 완료");
            }
//            for (Element article : mainArticles) {
//                //개별 기사 제목
//                String title = article.select("strong").text().trim();
//
//                // 기사 url
//                String articleUrl = article.selectFirst(".press_news_link").attr("href");
//
//                // 기사 내용
//                Document contentDocument = Jsoup.connect(articleUrl).timeout(5000).get();
//                String content = contentDocument.select(".story-news.article").text();
//                //DB에 저장하는 로직
//                NewsEntity newsEntity = NewsEntity.builder()
//                        .id((long)mainArticles.indexOf(article)+1)
//                        .title(title)
//                        .content(content)
//                        .build();
//                newsRepository.save(newsEntity);
//                log.info("뉴스 크롤링 완료");
//            }
        }catch (Exception e){
            log.error(e.getMessage());
            throw new NewsException(ErrorCode.NEWS_CRAWLING_FAILED);
        }
    }

}
