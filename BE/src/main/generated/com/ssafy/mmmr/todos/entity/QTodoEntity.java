package com.ssafy.mmmr.todos.entity;

import static com.querydsl.core.types.PathMetadataFactory.*;

import com.querydsl.core.types.dsl.*;

import com.querydsl.core.types.PathMetadata;
import javax.annotation.processing.Generated;
import com.querydsl.core.types.Path;
import com.querydsl.core.types.dsl.PathInits;


/**
 * QTodoEntity is a Querydsl query type for TodoEntity
 */
@Generated("com.querydsl.codegen.DefaultEntitySerializer")
public class QTodoEntity extends EntityPathBase<TodoEntity> {

    private static final long serialVersionUID = -1278141167L;

    private static final PathInits INITS = PathInits.DIRECT2;

    public static final QTodoEntity todoEntity = new QTodoEntity("todoEntity");

    public final StringPath content = createString("content");

    public final DateTimePath<java.time.LocalDateTime> createdAt = createDateTime("createdAt", java.time.LocalDateTime.class);

    public final BooleanPath deleted = createBoolean("deleted");

    public final NumberPath<Long> id = createNumber("id", Long.class);

    public final BooleanPath isDone = createBoolean("isDone");

    public final com.ssafy.mmmr.profiles.entity.QProfileEntity profile;

    public final DateTimePath<java.time.LocalDateTime> updatedAt = createDateTime("updatedAt", java.time.LocalDateTime.class);

    public QTodoEntity(String variable) {
        this(TodoEntity.class, forVariable(variable), INITS);
    }

    public QTodoEntity(Path<? extends TodoEntity> path) {
        this(path.getType(), path.getMetadata(), PathInits.getFor(path.getMetadata(), INITS));
    }

    public QTodoEntity(PathMetadata metadata) {
        this(metadata, PathInits.getFor(metadata, INITS));
    }

    public QTodoEntity(PathMetadata metadata, PathInits inits) {
        this(TodoEntity.class, metadata, inits);
    }

    public QTodoEntity(Class<? extends TodoEntity> type, PathMetadata metadata, PathInits inits) {
        super(type, metadata, inits);
        this.profile = inits.isInitialized("profile") ? new com.ssafy.mmmr.profiles.entity.QProfileEntity(forProperty("profile"), inits.get("profile")) : null;
    }

}

